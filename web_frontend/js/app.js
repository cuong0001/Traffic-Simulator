// Illustrated map renderer (static — no vehicles yet).
//
// Architecture ported from the reference project and adapted to our map.json:
//   - a CACHED STATIC LAYER (ground, buildings, road ribbons, trees, stop-lines)
//     drawn to an offscreen canvas and blitted each frame;
//   - rebuilt INCREMENTALLY in ~6 ms slices across frames (a full repaint is
//     10k+ strokes on a city map) with viewport culling + a zoom LOD;
//   - when the camera moves, the stale layer is blitted transformed (cheap,
//     slightly blurry) while a crisp rebuild runs in the background.
// Scenery (trees) is scattered deterministically into road-free, building-free
// gaps via an occupancy grid, so it coexists with the real OSM footprints.

"use strict";

import { loadMap } from "./api.js";
import { Sprites } from "./sprites.js";

// ---- palette ----
const C = {
  grass: "#7ca55e",
  sidewalk: "#d8d2b8",
  asphalt: "#3a3f47",
  dash: "rgba(232,227,205,.7)",
  bldShadow: "rgba(18,22,30,.35)",
  bldEdge: "rgba(70,64,52,.55)",
  signalHead: "#161b25",
  stopLine: "rgba(240,240,235,.85)",
};

// Building footprint fill by OSM `kind`.
const BLD_FILL = {
  school: "#d9c08a", university: "#d3ba82", college: "#d3ba82", kindergarten: "#d9c08a",
  commercial: "#a9bfd2", retail: "#a9bfd2", office: "#9fb6ca", supermarket: "#a9bfd2",
  industrial: "#c2b8a6", warehouse: "#c2b8a6", factory: "#c2b8a6",
  hospital: "#d7b0b0", church: "#c9c3d6", hotel: "#bdc9b6",
  apartments: "#c4bfa8", house: "#d3b48f", residential: "#cdbfa2", detached: "#d3b48f",
  default: "#cbc4b3",
};

const ROAD_W = 30;    // base ribbon width in world units (scaled by US)
const LANE_W = 3.5;   // meters
const LANE_VIS = 1.9; // visual exaggeration

// ---- globals ----
const canvas = document.getElementById("map");
const ctx = canvas.getContext("2d");
let staticLayer = document.createElement("canvas");
let spareLayer = null;

let mapData = null;
let nodeById = {};
let roadById = {};
let drawRoads = [];
let roadWOf = {};
let signalHeads = [];
let props = [];                       // scattered trees [{name, x, y, w}]
let world = { minX: 0, minY: 0, w: 1600, h: 920 };
let T = { s: 1, tx: 0, ty: 0 };       // world -> canvas-px transform
let paintedT = null;                  // transform the static bitmap was painted at
let rebuild = null;                   // in-flight sliced rebuild
let staticDirty = true;
let lastStaticPaint = 0;
let US = 1;
let roadW = ROAD_W;
let grassPattern = null;

const wx = (x) => x * T.s + T.tx;
const wy = (y) => y * T.s + T.ty;

// ---- small helpers ----
function toast(msg) {
  const el = document.getElementById("toast");
  el.textContent = msg;
  el.classList.remove("hidden");
  clearTimeout(toast._t);
  toast._t = setTimeout(() => el.classList.add("hidden"), 2600);
}

function mulberry32(a) {
  return function () {
    a |= 0; a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
function hashInt(id) {
  let h = 0;
  for (const ch of id) h = (h * 31 + ch.charCodeAt(0)) >>> 0;
  return h;
}

// ---- camera ----
function resize() {
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  canvas.width = Math.round(canvas.clientWidth * dpr);
  canvas.height = Math.round(canvas.clientHeight * dpr);
  staticLayer.width = canvas.width;
  staticLayer.height = canvas.height;
  paintedT = null;   // cached bitmap is gone
  rebuild = null;    // abort any in-flight rebuild (dims changed)
  fitView();
}

function fitView() {
  const margin = 48;
  const s = Math.min(
    (canvas.width - margin * 2) / world.w,
    (canvas.height - margin * 2) / world.h
  );
  T.s = s;
  T.tx = (canvas.width - world.w * s) / 2 - world.minX * s;
  T.ty = (canvas.height - world.h * s) / 2 - world.minY * s;
  staticDirty = true;
}

function initInput() {
  let dragging = false, lastX = 0, lastY = 0;
  const scaleX = () => canvas.width / canvas.clientWidth;
  const scaleY = () => canvas.height / canvas.clientHeight;

  canvas.addEventListener("pointerdown", (e) => {
    dragging = true; lastX = e.clientX; lastY = e.clientY;
    canvas.classList.add("dragging");
    canvas.setPointerCapture(e.pointerId);
  });
  canvas.addEventListener("pointermove", (e) => {
    if (!dragging) return;
    T.tx += (e.clientX - lastX) * scaleX();
    T.ty += (e.clientY - lastY) * scaleY();
    lastX = e.clientX; lastY = e.clientY;
  });
  const endDrag = (e) => {
    dragging = false;
    canvas.classList.remove("dragging");
    if (e.pointerId != null && canvas.hasPointerCapture?.(e.pointerId))
      canvas.releasePointerCapture(e.pointerId);
  };
  canvas.addEventListener("pointerup", endDrag);
  canvas.addEventListener("pointercancel", endDrag);

  canvas.addEventListener("wheel", (e) => {
    e.preventDefault();
    const r = canvas.getBoundingClientRect();
    const cx = (e.clientX - r.left) * scaleX();
    const cy = (e.clientY - r.top) * scaleY();
    const factor = e.deltaY > 0 ? 0.9 : 1.1;
    const ns = Math.max(0.02, Math.min(8, T.s * factor));
    // keep the world point under the cursor fixed
    T.tx = cx - (cx - T.tx) * (ns / T.s);
    T.ty = cy - (cy - T.ty) * (ns / T.s);
    T.s = ns;
  }, { passive: false });

  window.addEventListener("keydown", (e) => {
    if (e.key === "f" || e.key === "F") fitView();
    if (e.key === "p" || e.key === "P")
      document.getElementById("fpsmeter").classList.toggle("hidden");
  });
  window.addEventListener("resize", resize);
}

// ---------------------------------------------------------------------------
// Map preparation
// ---------------------------------------------------------------------------
function prepareMap() {
  nodeById = {};
  for (const n of mapData.intersections) nodeById[n.id] = n;
  roadById = {};
  for (const r of mapData.roads) roadById[r.id] = r;
  // draw each two-way street once (skip the "_r" reverse twin)
  drawRoads = mapData.roads.filter(
    (r) => !(r.id.endsWith("_r") && roadById[r.id.slice(0, -2)])
  );

  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
  const grow = (x, y) => {
    if (x < minX) minX = x; if (x > maxX) maxX = x;
    if (y < minY) minY = y; if (y > maxY) maxY = y;
  };
  for (const n of mapData.intersections) grow(n.x, n.y);
  for (const b of mapData.buildings || [])
    for (const [px, py] of b.footprint || []) grow(px, py);
  if (!isFinite(minX)) { minX = minY = 0; maxX = maxY = 1000; }

  // Unit scale from the median road length (true meters): OSM blocks are small.
  const lens = drawRoads.map((r) => r.length).sort((a, b) => a - b);
  const median = lens.length ? lens[Math.floor(lens.length / 2)] : 240;
  US = Math.max(0.15, Math.min(1, median / 240));
  roadW = ROAD_W * US;
  roadWOf = {};
  for (const r of drawRoads) {
    const total = (r.lanes || 2) * (roadById[r.id + "_r"] ? 2 : 1);
    roadWOf[r.id] = (roadW * Math.max(2, Math.min(6, total))) / 4;
  }

  const pad = 40;
  world = { minX: minX - pad, minY: minY - pad, w: maxX - minX + pad * 2, h: maxY - minY + pad * 2 };

  buildSignalHeads();
  buildOccupancyAndTrees(minX, minY, maxX, maxY);
  resize();
}

// ---- signals: a stop-line head per approach into a signalized junction ----
function buildSignalHeads() {
  signalHeads = [];
  const setback = roadW * 1.1 + 3;      // world units back from the junction
  for (const n of mapData.intersections) {
    if (!n.signal) continue;
    for (const r of drawRoads) {
      // approaches: a drawn road with this junction as an endpoint
      let a, b;
      if (r.to === n.id) { a = nodeById[r.from]; b = n; }
      else if (r.from === n.id) { a = nodeById[r.to]; b = n; } // reverse dir of a 2-way
      else continue;
      if (!a || !b) continue;
      const dx = b.x - a.x, dy = b.y - a.y;
      const len = Math.hypot(dx, dy);
      if (len < 1e-6) continue;
      const ux = dx / len, uy = dy / len;
      const back = Math.min(setback, len * 0.5);
      signalHeads.push({
        baseId: r.id,
        sx: b.x - ux * back, sy: b.y - uy * back, // stop-line center on the road axis
        px: -uy, py: ux,                          // unit perpendicular
      });
    }
  }
}

// ---- occupancy grid + deterministic tree scatter ----
// Trees only go where there is neither road nor building. A coarse grid marks
// occupied cells once (O(roads + buildings)), then scatter is O(candidates).
let occ = null, occCols = 0, occRows = 0, occCell = 1;

function buildOccupancyAndTrees(minX, minY, maxX, maxY) {
  props = [];
  occCell = Math.max(14, roadW * 1.6);
  occCols = Math.max(1, Math.ceil((maxX - minX) / occCell) + 1);
  occRows = Math.max(1, Math.ceil((maxY - minY) / occCell) + 1);
  if (occCols * occRows > 4000000) { occ = null; return; } // safety cap
  occ = new Uint8Array(occCols * occRows);

  const cellX = (x) => Math.floor((x - minX) / occCell);
  const cellY = (y) => Math.floor((y - minY) / occCell);
  const mark = (cx, cy) => {
    if (cx < 0 || cy < 0 || cx >= occCols || cy >= occRows) return;
    occ[cy * occCols + cx] = 1;
  };

  // stamp roads (sampled along each segment, 1-ring radius)
  for (const r of drawRoads) {
    const a = nodeById[r.from], b = nodeById[r.to];
    if (!a || !b) continue;
    const steps = Math.max(1, Math.ceil(Math.hypot(b.x - a.x, b.y - a.y) / (occCell * 0.6)));
    for (let i = 0; i <= steps; i++) {
      const t = i / steps;
      const cx = cellX(a.x + (b.x - a.x) * t), cy = cellY(a.y + (b.y - a.y) * t);
      for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) mark(cx + dx, cy + dy);
    }
  }
  // stamp building footprint bounding boxes (conservative: trees avoid buildings)
  for (const bld of mapData.buildings || []) {
    const fp = bld.footprint || [];
    if (fp.length < 3) continue;
    let bxMin = Infinity, byMin = Infinity, bxMax = -Infinity, byMax = -Infinity;
    for (const [px, py] of fp) {
      if (px < bxMin) bxMin = px; if (px > bxMax) bxMax = px;
      if (py < byMin) byMin = py; if (py > byMax) byMax = py;
    }
    for (let cy = cellY(byMin); cy <= cellY(byMax); cy++)
      for (let cx = cellX(bxMin); cx <= cellX(bxMax); cx++) mark(cx, cy);
  }

  // scatter trees on a jittered grid, only in free cells, deterministic + capped
  const rng = mulberry32(20260719);
  const spacing = occCell * 2.4;
  const maxTrees = 1400;
  const treeW = Math.max(6, 9 * US * 4);   // world-unit canopy width
  for (let y = minY; y < maxY && props.length < maxTrees; y += spacing) {
    for (let x = minX; x < maxX && props.length < maxTrees; x += spacing) {
      if (rng() < 0.35) continue;                       // thin it out
      const jx = x + (rng() - 0.5) * spacing * 0.7;
      const jy = y + (rng() - 0.5) * spacing * 0.7;
      const cx = cellX(jx), cy = cellY(jy);
      if (cx < 1 || cy < 1 || cx >= occCols - 1 || cy >= occRows - 1) continue;
      // require the cell and its 4-neighbours free (tree body has room)
      if (occ[cy * occCols + cx] || occ[cy * occCols + cx - 1] || occ[cy * occCols + cx + 1] ||
          occ[(cy - 1) * occCols + cx] || occ[(cy + 1) * occCols + cx]) continue;
      const seed = hashInt(`${cx},${cy}`);
      props.push({ name: "tree-" + (1 + (seed % 4)), x: jx, y: jy, w: treeW * (0.8 + rng() * 0.5) });
    }
  }
  props.sort((a, b) => a.y - b.y);   // painter's order
}

// ---------------------------------------------------------------------------
// Sprite drawing (base-anchored) with procedural fallbacks
// ---------------------------------------------------------------------------
function propShadow(g, sx, sy, w) {
  const grad = g.createRadialGradient(sx, sy, w * 0.05, sx, sy, w * 0.42);
  grad.addColorStop(0, "rgba(18,26,38,.28)");
  grad.addColorStop(1, "rgba(18,26,38,0)");
  g.fillStyle = grad;
  g.beginPath();
  g.ellipse(sx, sy, w * 0.42, w * 0.15, 0, 0, Math.PI * 2);
  g.fill();
}

function drawTreeFallback(g, sx, sy, w) {
  g.fillStyle = "#6b4a2f";
  g.fillRect(sx - w * 0.06, sy - w * 0.4, w * 0.12, w * 0.4);
  g.fillStyle = "#4f8040";
  g.beginPath(); g.arc(sx, sy - w * 0.62, w * 0.42, 0, 7); g.fill();
  g.beginPath(); g.arc(sx - w * 0.24, sy - w * 0.42, w * 0.3, 0, 7); g.fill();
  g.beginPath(); g.arc(sx + w * 0.24, sy - w * 0.42, w * 0.3, 0, 7); g.fill();
}

function drawProp(g, p) {
  const sx = wx(p.x), sy = wy(p.y);
  const w = p.w * T.s;
  if (w < 2.5) return;
  propShadow(g, sx, sy, w);
  const spr = Sprites.get(p.name);
  if (spr) {
    const h = w * (spr.h / spr.w);
    g.drawImage(spr.img, sx - w / 2, sy - h + w * 0.06, w, h);
  } else {
    drawTreeFallback(g, sx, sy, w);
  }
}

function makeGroundPattern(g) {
  const spr = Sprites.get("ground-1") || Sprites.get("ground-2") || Sprites.get("ground-3");
  if (!spr) return null;
  const p = g.createPattern(spr.img, "repeat");
  if (p && p.setTransform) {
    const worldSize = 46;                 // world units per tile
    const k = (worldSize / spr.w) * T.s;
    p.setTransform(new DOMMatrix([k, 0, 0, k, T.tx, T.ty]));
  }
  return p;
}

// ---------------------------------------------------------------------------
// Static layer: incremental, culled, LOD rebuild
// ---------------------------------------------------------------------------
function viewCull(snap) {
  const pad = roadW * 3 + 60;
  return {
    minX: (0 - snap.tx) / snap.s - pad,
    minY: (0 - snap.ty) / snap.s - pad,
    maxX: (canvas.width - snap.tx) / snap.s + pad,
    maxY: (canvas.height - snap.ty) / snap.s + pad,
  };
}
function segVisible(a, b, c) {
  return !(Math.max(a.x, b.x) < c.minX || Math.min(a.x, b.x) > c.maxX ||
           Math.max(a.y, b.y) < c.minY || Math.min(a.y, b.y) > c.maxY);
}

function beginRebuild() {
  if (!spareLayer) spareLayer = document.createElement("canvas");
  if (spareLayer.width !== canvas.width) spareLayer.width = canvas.width;
  if (spareLayer.height !== canvas.height) spareLayer.height = canvas.height;
  rebuild = {
    g: spareLayer.getContext("2d"),
    phase: 0, cursor: 0,
    snap: { s: T.s, tx: T.tx, ty: T.ty },
    lod: T.s < 0.4,          // far zoom: no lane dashes, no outlines
  };
  rebuild.cull = viewCull(rebuild.snap);
}

const REBUILD_PHASES = 5;

function runPhase(rb, deadline) {
  const g = rb.g, cull = rb.cull;
  switch (rb.phase) {
    case 0: {  // ground
      g.clearRect(0, 0, spareLayer.width, spareLayer.height);
      grassPattern = makeGroundPattern(g);
      g.fillStyle = grassPattern || C.grass;
      g.fillRect(0, 0, spareLayer.width, spareLayer.height);
      return true;
    }
    case 1: {  // building footprints (2.5D: drop shadow + fill + edge)
      const list = mapData.buildings || [];
      const off = Math.max(1, 2.2 * US * T.s);   // shadow offset (px)
      for (; rb.cursor < list.length; rb.cursor++) {
        if ((rb.cursor & 31) === 0 && performance.now() > deadline) return false;
        const b = list[rb.cursor];
        const fp = b.footprint || [];
        if (fp.length < 3) continue;
        if (fp[0][0] < cull.minX - 200 || fp[0][0] > cull.maxX + 200 ||
            fp[0][1] < cull.minY - 200 || fp[0][1] > cull.maxY + 200) continue;
        const trace = (dx, dy) => {
          g.beginPath();
          g.moveTo(wx(fp[0][0]) + dx, wy(fp[0][1]) + dy);
          for (let i = 1; i < fp.length; i++) g.lineTo(wx(fp[i][0]) + dx, wy(fp[i][1]) + dy);
          g.closePath();
        };
        if (!rb.lod) { trace(off, off); g.fillStyle = C.bldShadow; g.fill(); }
        trace(0, 0);
        g.fillStyle = BLD_FILL[b.kind] || BLD_FILL.default;
        g.fill();
        if (!rb.lod) {
          g.strokeStyle = C.bldEdge;
          g.lineWidth = Math.max(1, 1.1 * US * T.s);
          g.stroke();
        }
      }
      rb.cursor = 0;
      return true;
    }
    case 2: {  // sidewalk ribbons (under the asphalt)
      g.lineCap = rb.lod ? "butt" : "round";
      g.strokeStyle = C.sidewalk;
      for (; rb.cursor < drawRoads.length; rb.cursor++) {
        if ((rb.cursor & 63) === 0 && performance.now() > deadline) return false;
        const r = drawRoads[rb.cursor];
        const a = nodeById[r.from], b = nodeById[r.to];
        if (!a || !b || !segVisible(a, b, cull)) continue;
        g.lineWidth = ((roadWOf[r.id] || roadW) + 8 * US) * T.s;
        g.beginPath(); g.moveTo(wx(a.x), wy(a.y)); g.lineTo(wx(b.x), wy(b.y)); g.stroke();
      }
      rb.cursor = 0;
      return true;
    }
    case 3: {  // asphalt + optional lane markings
      const laneDetail = !rb.lod && T.s >= 1.1;   // dashes only when zoomed in
      g.lineCap = rb.lod ? "butt" : "round";
      for (; rb.cursor < drawRoads.length; rb.cursor++) {
        if ((rb.cursor & 63) === 0 && performance.now() > deadline) return false;
        const r = drawRoads[rb.cursor];
        const a = nodeById[r.from], b = nodeById[r.to];
        if (!a || !b || !segVisible(a, b, cull)) continue;
        const x1 = wx(a.x), y1 = wy(a.y), x2 = wx(b.x), y2 = wy(b.y);
        g.strokeStyle = C.asphalt;
        g.lineWidth = (roadWOf[r.id] || roadW) * T.s;
        g.beginPath(); g.moveTo(x1, y1); g.lineTo(x2, y2); g.stroke();
        if (!laneDetail) continue;
        const len = Math.hypot(x2 - x1, y2 - y1) || 1;
        const px = -(y2 - y1) / len, py = (x2 - x1) / len;
        const bound = LANE_W * LANE_VIS * US * T.s;
        g.strokeStyle = C.dash;
        g.setLineDash([9 * US * T.s, 13 * US * T.s]);
        g.lineWidth = Math.max(1, 1.4 * US * T.s);
        for (const o of [bound, -bound]) {
          g.beginPath();
          g.moveTo(x1 + px * o, y1 + py * o);
          g.lineTo(x2 + px * o, y2 + py * o);
          g.stroke();
        }
        g.setLineDash([]);
      }
      rb.cursor = 0;
      return true;
    }
    case 4: {  // stop-lines + trees
      if (!rb.lod) {
        g.strokeStyle = C.stopLine;
        for (const h of signalHeads) {
          if (h.sx < cull.minX || h.sx > cull.maxX || h.sy < cull.minY || h.sy > cull.maxY) continue;
          const w = (roadWOf[h.baseId] || roadW) / 2;
          g.lineWidth = Math.max(1, 1.2 * US * T.s);
          g.beginPath();
          g.moveTo(wx(h.sx - h.px * w), wy(h.sy - h.py * w));
          g.lineTo(wx(h.sx + h.px * w), wy(h.sy + h.py * w));
          g.stroke();
        }
      }
      for (; rb.cursor < props.length; rb.cursor++) {
        if ((rb.cursor & 63) === 0 && performance.now() > deadline) return false;
        const p = props[rb.cursor];
        if (p.x < cull.minX - p.w * 3 || p.x > cull.maxX + p.w * 3 ||
            p.y < cull.minY - p.w * 3 || p.y > cull.maxY + p.w * 3) continue;
        drawProp(g, p);
      }
      rb.cursor = 0;
      return true;
    }
  }
  return true;
}

function stepRebuild(budgetMs) {
  if (!rebuild) return;
  const deadline = performance.now() + budgetMs;
  const saved = T;
  T = rebuild.snap;                 // draw in the snapshot's transform
  let finished = false;
  while (rebuild.phase < REBUILD_PHASES) {
    if (!runPhase(rebuild, deadline)) break;   // budget exhausted mid-phase
    rebuild.phase++;
    if (rebuild.phase >= REBUILD_PHASES) { finished = true; break; }
    if (performance.now() > deadline) break;
  }
  T = saved;
  if (finished) {
    const done = spareLayer;
    spareLayer = staticLayer;
    staticLayer = done;             // swap the freshly painted layer in
    paintedT = rebuild.snap;
    lastStaticPaint = performance.now();
    rebuild = null;
  }
}

function drawStaticLayer() {
  const now = performance.now();
  if (!paintedT) { beginRebuild(); stepRebuild(1e9); }   // first frame: synchronous
  if (staticDirty && !rebuild && now - lastStaticPaint > 140) {
    staticDirty = false;
    beginRebuild();
  }
  if (rebuild) stepRebuild(6);      // one ~6 ms slice per frame

  if (paintedT.s === T.s && paintedT.tx === T.tx && paintedT.ty === T.ty) {
    ctx.drawImage(staticLayer, 0, 0);
  } else {
    const k = T.s / paintedT.s;      // transform the stale bitmap to the live camera
    ctx.save();
    ctx.setTransform(k, 0, 0, k, T.tx - paintedT.tx * k, T.ty - paintedT.ty * k);
    ctx.drawImage(staticLayer, 0, 0);
    ctx.restore();
    staticDirty = true;             // schedule a crisp repaint
  }
}

// signal heads (dynamic layer — fixed neutral color; the sim PR animates them)
function drawSignals() {
  const dot = Math.max(1.5, 2.2 * US * T.s);
  const rMinX = -T.tx / T.s, rMaxX = (canvas.width - T.tx) / T.s;
  const rMinY = -T.ty / T.s, rMaxY = (canvas.height - T.ty) / T.s;
  ctx.fillStyle = C.signalHead;
  for (const h of signalHeads) {
    if (h.sx < rMinX - 30 || h.sx > rMaxX + 30 || h.sy < rMinY - 30 || h.sy > rMaxY + 30) continue;
    const off = (roadWOf[h.baseId] || roadW) / 2 + 2.5 * US;
    ctx.beginPath();
    ctx.arc(wx(h.sx + h.px * off), wy(h.sy + h.py * off), dot, 0, Math.PI * 2);
    ctx.fill();
  }
}

// ---- render loop + fps meter ----
let fpsFrames = 0, fpsWorst = 0, fpsWindowStart = 0, lastFrameTs = 0;
function render() {
  const now = performance.now();
  if (lastFrameTs) {
    const gap = now - lastFrameTs;
    if (gap > fpsWorst) fpsWorst = gap;
    fpsFrames++;
    if (now - fpsWindowStart > 500) {
      const meter = document.getElementById("fpsmeter");
      if (meter && !meter.classList.contains("hidden")) {
        const fps = Math.round((fpsFrames * 1000) / (now - fpsWindowStart));
        meter.textContent = `${fps} fps · worst ${Math.round(fpsWorst)} ms · zoom ${T.s.toFixed(2)}`;
      }
      fpsFrames = 0; fpsWorst = 0; fpsWindowStart = now;
    }
  } else { fpsWindowStart = now; }
  lastFrameTs = now;

  if (mapData) {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    drawStaticLayer();
    drawSignals();
  }
  requestAnimationFrame(render);
}

// ---------------------------------------------------------------------------
// Boot
// ---------------------------------------------------------------------------
function showError(msg) {
  const overlay = document.getElementById("loading");
  overlay.classList.remove("hidden");
  overlay.classList.add("error");
  document.getElementById("loading-text").textContent = msg;
}

async function boot() {
  initInput();
  try {
    const [data] = await Promise.all([loadMap(), Sprites.load()]);
    mapData = data;
    prepareMap();
    document.getElementById("hud-stats").textContent =
      `${mapData.intersections.length.toLocaleString()} nodes · ` +
      `${drawRoads.length.toLocaleString()} roads · ` +
      `${(mapData.buildings || []).length.toLocaleString()} buildings · ` +
      `${props.length.toLocaleString()} trees`;
    document.getElementById("loading").classList.add("hidden");
    requestAnimationFrame(render);
  } catch (err) {
    console.error(err);
    showError(String(err.message || err));
    toast("Failed to load the map");
  }
}

boot();
