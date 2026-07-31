// Illustrated map renderer + live simulation layer.
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
//
// On top of that sits a DYNAMIC LAYER redrawn every frame: vehicles (position
// interpolated from /api/state at 4 Hz), live signal colours, and congested or
// closed roads. When the simulation server is absent the static map still
// renders on its own and the control panel stays hidden.

"use strict";

import { loadMap, loadState } from "./api.js";
import { Sprites } from "./sprites.js";
import * as UI from "./ui.js";

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
let fitScale = 1;                     // scale at which the whole city fits on screen
let paintedT = null;                  // transform the static bitmap was painted at
let rebuild = null;                   // in-flight sliced rebuild
let staticDirty = true;
let lastStaticPaint = 0;
let US = 1;
let roadW = ROAD_W;
let grassPattern = null;
let dragMoved = false;                // true when a pointer-down turned into a pan

const wx = (x) => x * T.s + T.tx;
const wy = (y) => y * T.s + T.ty;

// ---- small helpers ----

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
  fitScale = s;      // zoom limits are expressed relative to this, see the wheel handler
  T.s = s;
  T.tx = (canvas.width - world.w * s) / 2 - world.minX * s;
  T.ty = (canvas.height - world.h * s) / 2 - world.minY * s;
  staticDirty = true;
}

function initInput() {
  let dragging = false, lastX = 0, lastY = 0, downX = 0, downY = 0;
  const scaleX = () => canvas.width / canvas.clientWidth;
  const scaleY = () => canvas.height / canvas.clientHeight;

  canvas.addEventListener("pointerdown", (e) => {
    dragging = true; lastX = e.clientX; lastY = e.clientY;
    downX = e.clientX; downY = e.clientY; dragMoved = false;
    canvas.classList.add("dragging");
    canvas.setPointerCapture(e.pointerId);
  });
  canvas.addEventListener("pointermove", (e) => {
    if (!dragging) return;
    T.tx += (e.clientX - lastX) * scaleX();
    T.ty += (e.clientY - lastY) * scaleY();
    lastX = e.clientX; lastY = e.clientY;
    // a few px of slop so a click with a shaky hand still counts as a click
    if (Math.hypot(e.clientX - downX, e.clientY - downY) > 4) dragMoved = true;
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
    const factor = e.deltaY > 0 ? 1 / 1.15 : 1.15;
    // Limits are relative to the fit view, not absolute, because T.s is in
    // canvas pixels per world METRE and that varies hugely by map: a fixed cap
    // that reaches street level on a small map barely leaves the district view
    // on a real OSM export. 25 px/m is the floor for the deepest zoom so a 4.5 m
    // car is always reachable at ~110 px however large the city is.
    const maxS = Math.max(fitScale * 9, 25);
    const ns = Math.max(fitScale * 0.5, Math.min(maxS, T.s * factor));
    // keep the world point under the cursor fixed
    T.tx = cx - (cx - T.tx) * (ns / T.s);
    T.ty = cy - (cy - T.ty) * (ns / T.s);
    T.s = ns;
  }, { passive: false });

  // Double-click to frame the whole city again -- but not while an editing tool
  // is armed, where the second click is a placement, not a request to zoom out.
  canvas.addEventListener("dblclick", () => {
    if (placingProp || UI.isArmed()) return;
    fitView();
  });

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
        nodeId: n.id,                             // to look up the live light state
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
  const spr = Sprites.get("ground");
  if (!spr) return null;
  const p = g.createPattern(spr.img, "repeat");
  if (p && p.setTransform) {
    const worldSize = 110;                // world units one ground tile spans (tune here)
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

// ---------------------------------------------------------------------------
// Dynamic layer: live simulation (vehicles, signal colours, congestion)
// ---------------------------------------------------------------------------
const LIGHT_COLOR = ["#ef4444", "#f59e0b", "#22c55e"];   // 0 red, 1 amber, 2 green
const VEH_LEN = { car: 4.5, bus: 6.5, emergency: 5.0, motorbike: 2.0 };

let simState = null;            // last snapshot from /api/state
let lightState = {};            // nodeId -> 0|1|2
let roadState = {};             // roadId -> { congestion, closed }
let vehIndex = new Map();       // id -> { x, y, h, tx, ty, th, type }
let simOnline = false;

// Place a vehicle from (road id, metres along it). Roads are straight segments
// between two nodes — the loader already split curves into waypoint nodes — so
// this is a plain lerp. Returns null when the road/nodes are unknown.
function placeOnRoad(roadId, pos, lane = 0, lat = 0) {
  const r = roadById[roadId];
  if (!r) return null;
  const a = nodeById[r.from], b = nodeById[r.to];
  if (!a || !b) return null;
  const dx = b.x - a.x, dy = b.y - a.y;
  const len = Math.hypot(dx, dy) || 1;
  const t = Math.max(0, Math.min(1, (pos || 0) / (r.length || len)));
  const ux = dx / len, uy = dy / len;

  // Lateral placement: keep right of the centre line so opposing directions
  // don't overlap, then step out by the lane index. `lat` is the in-progress
  // lane-change drift the backend reports, in metres.
  const base = r.id.endsWith("_r") ? r.id.slice(0, -2) : r.id;
  const halfW = (roadWOf[base] || roadW) * 0.5;
  const lanes = Math.max(1, r.lanes || 1);
  const laneW = halfW / lanes;
  const off = laneW * (lane + 0.5) + (lat || 0) * US;

  return {
    x: a.x + dx * t - uy * off,
    y: a.y + dy * t + ux * off,
    h: Math.atan2(dy, dx),
  };
}

function applyState(s) {
  simState = s;
  lightState = {};
  for (const l of s.lights || []) lightState[l.id] = l.state;
  roadState = {};
  for (const r of s.roads || []) roadState[r.id] = r;

  const seen = new Set();
  for (const v of s.vehicles || []) {
    const p = placeOnRoad(v.road, v.pos, v.lane || 0, v.lat || 0);
    if (!p) continue;
    seen.add(v.id);
    let e = vehIndex.get(v.id);
    if (!e) {
      // new vehicle: appear where it actually is, don't glide in from nowhere
      e = { x: p.x, y: p.y, h: p.h, tx: p.x, ty: p.y, th: p.h, type: v.type };
      vehIndex.set(v.id, e);
    } else {
      // a road change teleports the target; snap instead of sliding across the map
      if (Math.hypot(p.x - e.tx, p.y - e.ty) > 60) { e.x = p.x; e.y = p.y; e.h = p.h; }
      e.tx = p.x; e.ty = p.y; e.th = p.h; e.type = v.type;
    }
  }
  for (const id of vehIndex.keys()) if (!seen.has(id)) vehIndex.delete(id);
}

// Ease the drawn position toward the last reported one, so 4 Hz polling still
// looks continuous. k is frame-rate independent.
function advanceVehicles(dtMs) {
  const k = 1 - Math.pow(0.0025, dtMs / 1000);
  for (const e of vehIndex.values()) {
    e.x += (e.tx - e.x) * k;
    e.y += (e.ty - e.y) * k;
    let d = e.th - e.h;                       // shortest way round the circle
    while (d > Math.PI) d -= Math.PI * 2;
    while (d < -Math.PI) d += Math.PI * 2;
    e.h += d * k;
  }
}

function drawVehicles() {
  if (!vehIndex.size) return;
  const rMinX = -T.tx / T.s, rMaxX = (canvas.width - T.tx) / T.s;
  const rMinY = -T.ty / T.s, rMaxY = (canvas.height - T.ty) / T.s;

  for (const [id, e] of vehIndex) {
    if (e.x < rMinX - 40 || e.x > rMaxX + 40 || e.y < rMinY - 40 || e.y > rMaxY + 40) continue;

    const lenM = VEH_LEN[e.type] || 4.5;
    const w = Math.max(3, lenM * US * T.s * 1.6);   // on-screen length in px
    const h = w * 0.5;
    const sx = wx(e.x), sy = wy(e.y);

    const spr = e.type === "car"
      ? Sprites.pick("car", 4, hashInt(id))
      : Sprites.get(e.type + "-1");

    ctx.save();
    ctx.translate(sx, sy);
    ctx.rotate(e.h);
    if (spr) {
      ctx.drawImage(spr.img, -w / 2, -h / 2, w, h);
    } else {
      ctx.fillStyle = e.type === "emergency" ? "#ef4444"
                    : e.type === "bus" ? "#f59e0b" : "#e5e7eb";
      ctx.fillRect(-w / 2, -h / 2, w, h);
      ctx.fillStyle = "rgba(0,0,0,.35)";
      ctx.fillRect(w * 0.12, -h / 2, w * 0.26, h);   // windscreen hint
    }
    ctx.restore();
  }
}

// signal heads — coloured by the live light state when the sim is running
function drawSignals() {
  const dot = Math.max(1.5, 2.2 * US * T.s);
  const rMinX = -T.tx / T.s, rMaxX = (canvas.width - T.tx) / T.s;
  const rMinY = -T.ty / T.s, rMaxY = (canvas.height - T.ty) / T.s;
  for (const h of signalHeads) {
    if (h.sx < rMinX - 30 || h.sx > rMaxX + 30 || h.sy < rMinY - 30 || h.sy > rMaxY + 30) continue;
    const st = lightState[h.nodeId];
    ctx.fillStyle = st === undefined ? C.signalHead : LIGHT_COLOR[st] || C.signalHead;
    const off = (roadWOf[h.baseId] || roadW) / 2 + 2.5 * US;
    ctx.beginPath();
    ctx.arc(wx(h.sx + h.px * off), wy(h.sy + h.py * off), dot, 0, Math.PI * 2);
    ctx.fill();
  }
}

// Congested / closed roads, drawn over the asphalt.
function drawRoadStatus() {
  const ids = Object.keys(roadState);
  if (!ids.length) return;
  ctx.lineCap = "round";
  for (const id of ids) {
    const r = roadById[id];
    if (!r) continue;
    const a = nodeById[r.from], b = nodeById[r.to];
    if (!a || !b) continue;
    const st = roadState[id];
    const base = id.endsWith("_r") ? id.slice(0, -2) : id;
    ctx.strokeStyle = st.closed
      ? "rgba(239,68,68,.75)"
      : `rgba(245,158,11,${Math.min(0.6, 0.15 + st.congestion * 0.5)})`;
    ctx.lineWidth = Math.max(1.5, (roadWOf[base] || roadW) * T.s * 0.5);
    ctx.beginPath();
    ctx.moveTo(wx(a.x), wy(a.y));
    ctx.lineTo(wx(b.x), wy(b.y));
    ctx.stroke();
  }
}

// ---- render loop + fps meter ----
let fpsFrames = 0, fpsWorst = 0, fpsWindowStart = 0, lastFrameTs = 0;
function render() {
  const now = performance.now();
  const frameDt = lastFrameTs ? Math.min(100, now - lastFrameTs) : 16;
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
    advanceVehicles(frameDt);
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    drawStaticLayer();
    drawRoadStatus();
    drawSignals();
    drawVehicles();
  }
  requestAnimationFrame(render);
}

// ---------------------------------------------------------------------------
// Simulation polling
// ---------------------------------------------------------------------------
const POLL_MS = 250;             // 4 Hz; drawing interpolates between samples

async function poll() {
  const s = await loadState();
  if (s) {
    if (!simOnline) {
      simOnline = true;
      UI.log("Simulation connected");
    }
    applyState(s);
  } else if (simOnline) {
    simOnline = false;
    simState = null;
    UI.log("Simulation offline");
  }
  UI.update(simState);
}

function randomNode() {
  const a = mapData.intersections;
  return a[(Math.random() * a.length) | 0].id;
}
function randomRoad() {
  return drawRoads[(Math.random() * drawRoads.length) | 0].id;
}

// ---- click-to-inspect + scenery placement -------------------------------
let placingProp = null;

function setPlacing(name) {
  placingProp = name;
  canvas.classList.toggle("placing", !!name);
}

// What is under a world point? Priority: vehicle, junction, road, building.
// Thresholds are in screen pixels converted to world units, so picking feels
// the same at every zoom level.
function pickAt(wxp, wyp) {
  let best = null, bestD = 18 / T.s;
  for (const [id, e] of vehIndex) {
    const d = Math.hypot(e.x - wxp, e.y - wyp);
    if (d < bestD) { bestD = d; best = { kind: "vehicle", id, e }; }
  }
  if (best) return best;

  bestD = 16 / T.s;
  for (const n of mapData.intersections) {
    const d = Math.hypot(n.x - wxp, n.y - wyp);
    if (d < bestD) { bestD = d; best = { kind: "junction", id: n.id, node: n }; }
  }
  if (best) return best;

  bestD = 20 / T.s;
  for (const r of drawRoads) {
    const a = nodeById[r.from], b = nodeById[r.to];
    if (!a || !b) continue;
    const vx = b.x - a.x, vy = b.y - a.y;
    const L2 = vx * vx + vy * vy || 1;
    let t = ((wxp - a.x) * vx + (wyp - a.y) * vy) / L2;
    t = Math.max(0, Math.min(1, t));
    const d = Math.hypot(a.x + vx * t - wxp, a.y + vy * t - wyp);
    if (d < bestD) { bestD = d; best = { kind: "road", id: r.id, road: r }; }
  }
  if (best) return best;

  for (const b of mapData.buildings || []) {
    const fp = b.footprint || [];
    if (fp.length < 3) continue;
    let inside = false;                       // ray cast
    for (let i = 0, j = fp.length - 1; i < fp.length; j = i++) {
      const [xi, yi] = fp[i], [xj, yj] = fp[j];
      if ((yi > wyp) !== (yj > wyp) &&
          wxp < ((xj - xi) * (wyp - yi)) / (yj - yi || 1e-9) + xi) inside = !inside;
    }
    if (inside) return { kind: "building", id: b.name, building: b };
  }
  return null;
}

// Nearest tree/grass patch, for "Delete Decoration".
function pickProp(wxp, wyp) {
  let idx = -1, bestD = 20 / T.s;
  for (let i = 0; i < props.length; i++) {
    const d = Math.hypot(props[i].x - wxp, props[i].y - wyp);
    if (d < bestD) { bestD = d; idx = i; }
  }
  return idx;
}

function describe(hit) {
  if (hit.kind === "vehicle") {
    const v = (simState?.vehicles || []).find((x) => x.id === hit.id) || {};
    return [`Vehicle ${hit.id}`, [
      ["Type", v.type || hit.e.type],
      ["Speed", `${((v.speed || 0) * 3.6).toFixed(0)} km/h`],
      ["Road", v.road || "—"],
      ["Position", `${(v.pos || 0).toFixed(1)} m`],
    ]];
  }
  if (hit.kind === "junction") {
    const n = hit.node;
    const deg = drawRoads.filter((r) => r.from === n.id || r.to === n.id).length;
    return [`Junction ${n.id}`, [
      ["Signalized", n.signal ? "yes" : "no"],
      ["Light", lightState[n.id] === undefined ? "—" : ["red", "amber", "green"][lightState[n.id]]],
      ["Roads", deg],
      ["Zone", n.zone || "—"],
    ]];
  }
  if (hit.kind === "road") {
    const r = hit.road, st = roadState[r.id] || {};
    return [`Road ${r.id}`, [
      ["From", r.from], ["To", r.to],
      ["Length", `${(r.length || 0).toFixed(0)} m`],
      ["Speed", `${r.speed_kmh || "?"} km/h`],
      ["Lanes", r.lanes ?? "?"],
      ["Congestion", `${Math.round((st.congestion || 0) * 100)}%`],
      ["Closed", st.closed ? "yes" : "no"],
    ]];
  }
  const b = hit.building;
  return [`Building ${b.name}`, [
    ["Kind", b.kind || "—"], ["Levels", b.levels ?? 1],
  ]];
}

function onMapClick(worldX, worldY) {
  // Scenery placement is client-side and takes priority over everything else.
  if (placingProp) {
    props.push({ name: placingProp, x: worldX, y: worldY,
                 w: placingProp.startsWith("grass") ? 26 : 16 });
    props.sort((p, q) => p.y - q.y);
    staticDirty = true;
    UI.log(`Placed ${placingProp}`);
    return;
  }

  const hit = pickAt(worldX, worldY);
  // An editing tool is armed: hand the click to the UI instead of inspecting.
  if (UI.handleMapClick(worldX, worldY, hit, pickProp(worldX, worldY))) return;

  if (!hit) { UI.hideSelection(); return; }
  const [title, rows] = describe(hit);
  UI.showSelection(title, rows);
}

// Remove a client-side prop by index (used by "Delete Decoration").
function removePropAt(i) {
  if (i < 0 || i >= props.length) return false;
  props.splice(i, 1);
  staticDirty = true;
  return true;
}

// Re-fetch the map after a server-side edit and rebuild the static layer.
async function reloadMap() {
  mapData = await loadMap();
  const keep = props.slice();       // client-side scenery survives a reload
  prepareMap();
  props = keep.concat(props);
  props.sort((p, q) => p.y - q.y);
  staticDirty = true;
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
  canvas.addEventListener("click", (e) => {
    if (dragMoved) return;                       // a pan, not a click
    const r = canvas.getBoundingClientRect();
    const px = (e.clientX - r.left) * (canvas.width / r.width);
    const py = (e.clientY - r.top) * (canvas.height / r.height);
    onMapClick((px - T.tx) / T.s, (py - T.ty) / T.s);
  });

  try {
    const [data] = await Promise.all([loadMap(), Sprites.load()]);
    mapData = data;
    prepareMap();
    document.getElementById("loading").classList.add("hidden");

    UI.init({
      fitView,
      poll,
      randomNode,
      randomRoad,
      setPlacing,
      removePropAt,
      reloadMap,
      getState: () => simState,
      getMap: () => mapData,
      getDrawRoads: () => drawRoads,
      getPropCount: () => props.length,
    });

    poll();
    setInterval(poll, POLL_MS);
    requestAnimationFrame(render);
  } catch (err) {
    console.error(err);
    showError(String(err.message || err));
    UI.toast("Failed to load the map");
  }
}

boot();
