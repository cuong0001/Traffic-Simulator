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

import { loadMap, loadState, Sim } from "./api.js";
import { Sprites } from "./sprites.js";
import * as UI from "./ui.js";
import { OsmPicker, importArea } from "./osm.js";

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
// Unit scale, derived from the median road length: the hand-drawn downtown map
// has ~240 m blocks, a real OSM extract has ~27 m ones. Everything measured in
// world units (road ribbon, scenery) shrinks by US. Vehicles use VS instead,
// which has a higher floor -- a car drawn to true scale on a city map is a
// speck, and the point of the sprite is to be identifiable.
let US = 1;
let VS = 1;
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
// True until the user first pans or zooms. While it holds, a resize refits;
// afterwards a resize keeps whatever the user was looking at.
let cameraUntouched = true;

function resize() {
  // What is at the middle of the screen right now, so we can put it back.
  const hadView = canvas.width > 0 && canvas.height > 0 && !cameraUntouched;
  const midX = hadView ? (canvas.width / 2 - T.tx) / T.s : 0;
  const midY = hadView ? (canvas.height / 2 - T.ty) / T.s : 0;
  const keptScale = T.s;

  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  canvas.width = Math.round(canvas.clientWidth * dpr);
  canvas.height = Math.round(canvas.clientHeight * dpr);
  staticLayer.width = canvas.width;
  staticLayer.height = canvas.height;
  paintedT = null;   // cached bitmap is gone
  rebuild = null;    // abort any in-flight rebuild (dims changed)

  fitView();         // always: it is what recomputes fitScale for the new size
  if (hadView) {
    // Restore the user's view. A window resize changes how much fits on
    // screen, not what the user was looking at.
    T.s = keptScale;
    T.tx = canvas.width / 2 - midX * keptScale;
    T.ty = canvas.height / 2 - midY * keptScale;
    // fitView() just cleared this; the camera is emphatically not at the fit
    // view, and without putting it back the NEXT resize would refit and throw
    // the view away after all.
    cameraUntouched = false;
  }
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
  cameraUntouched = true;
  staticDirty = true;
}

let hoverWorld = null;          // cursor in world units, for the add-road preview

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
    // Track the cursor in world units whether or not a drag is in progress:
    // the add-road preview needs it every frame, not only while panning.
    const r0 = canvas.getBoundingClientRect();
    hoverWorld = {
      x: ((e.clientX - r0.left) * scaleX() - T.tx) / T.s,
      y: ((e.clientY - r0.top) * scaleY() - T.ty) / T.s,
    };
    if (!dragging) return;
    T.tx += (e.clientX - lastX) * scaleX();
    T.ty += (e.clientY - lastY) * scaleY();
    cameraUntouched = false;
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
    // One mouse notch is one event, but a trackpad sends a stream of small
    // ones, so a fixed step per event flies past the target on a laptop.
    // Normalise to pixels first (deltaMode 1 = lines, 2 = pages), then clamp
    // so no single event can jump more than a notch and a bit.
    const px = e.deltaY * (e.deltaMode === 1 ? 16 : e.deltaMode === 2 ? 100 : 1);
    const factor = Math.min(1.25, Math.max(1 / 1.25, Math.pow(1.0018, -px)));
    // Limits are relative to the fit view, not absolute, because T.s is in
    // canvas pixels per world METRE and that varies hugely by map: a fixed cap
    // that reaches street level on a small map barely leaves the district view
    // on a real OSM export. 60 px/m is the floor for the deepest zoom, which
    // puts a 4.5 m car at ~270 px however large the city is.
    const maxS = Math.max(fitScale * 12, 60);
    const ns = Math.max(fitScale * 0.5, Math.min(maxS, T.s * factor));
    // keep the world point under the cursor fixed
    T.tx = cx - (cx - T.tx) * (ns / T.s);
    T.ty = cy - (cy - T.ty) * (ns / T.s);
    T.s = ns;
    cameraUntouched = false;
  }, { passive: false });

  // No dblclick handler on purpose. Fitting the view is a deliberate act --
  // "F" or Map -> Fit -- not something a stray second click should do while
  // you are working inside a junction you spent a while zooming into.

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
// The extent of everything drawable, and the `world` box fitView() frames.
function computeWorldBounds() {
  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
  const grow = (x, y) => {
    if (x < minX) minX = x; if (x > maxX) maxX = x;
    if (y < minY) minY = y; if (y > maxY) maxY = y;
  };
  for (const n of mapData.intersections) grow(n.x, n.y);
  for (const b of mapData.buildings || [])
    for (const [px, py] of b.footprint || []) grow(px, py);
  if (!isFinite(minX)) { minX = minY = 0; maxX = maxY = 1000; }

  const pad = 40;
  world = { minX: minX - pad, minY: minY - pad,
            w: maxX - minX + pad * 2, h: maxY - minY + pad * 2 };
  return { minX, minY, maxX, maxY };
}

// Everything derived from mapData that the CAMERA has no part in. A live edit
// runs only this, so zooming into a junction and then deleting a road there
// leaves the view exactly where it was instead of throwing it back to the whole
// city. Loading a different map runs prepareMap() below, which does refit.
function rebuildDerived() {
  nodeById = {};
  for (const n of mapData.intersections) nodeById[n.id] = n;
  roadById = {};
  for (const r of mapData.roads) roadById[r.id] = r;
  // draw each two-way street once (skip the "_r" reverse twin)
  drawRoads = mapData.roads.filter(
    (r) => !(r.id.endsWith("_r") && roadById[r.id.slice(0, -2)])
  );

  // /api/map emits one entry per physical street, but /api/state and
  // /api/vehicle/detail both speak in DIRECTED ids. Without the twins a third
  // of the fleet resolves to no road and is never drawn, a third of every
  // highlighted route is invisible, and two-way streets come out half width
  // because the lane count below cannot see the other direction. The twin of a
  // two-way street is an exact from/to swap with the same length and lanes
  // (checked against the reference server: 2783 pairs, no exceptions), so
  // mirroring them here is free where doubling the payload would not be.
  for (const r of mapData.roads) {
    if (r.oneway || r.id.endsWith("_r")) continue;
    const twin = r.id + "_r";
    if (!roadById[twin]) roadById[twin] = { ...r, id: twin, from: r.to, to: r.from };
  }

  // Unit scale from the median road length (true meters): OSM blocks are small.
  const lens = drawRoads.map((r) => r.length).sort((a, b) => a - b);
  const median = lens.length ? lens[Math.floor(lens.length / 2)] : 240;
  US = Math.max(0.2, Math.min(1, median / 240));
  VS = Math.max(US, 0.35);
  roadW = ROAD_W * US;
  roadWOf = {};
  for (const r of drawRoads) {
    const total = (r.lanes || 2) * (roadById[r.id + "_r"] ? 2 : 1);
    roadWOf[r.id] = (roadW * Math.max(2, Math.min(6, total))) / 4;
  }

  computeWorldBounds();
  buildSignalHeads();
  rebuildLiveRoads();
  staticDirty = true;
}

// Full initialisation for a NEW city: derived data, scenery, and a camera that
// frames it. Boot, Files -> Open and the OSM import go through here.
function prepareMap() {
  rebuildDerived();
  const b = computeWorldBounds();
  // A different city: there is no view worth preserving, so let resize() fit.
  cameraUntouched = true;

  // Procedural scenery is for the hand-drawn downtown map. A real extract
  // brings its own building footprints, and on a city-scale graph the scatter
  // is up to 1400 props each carrying a radial-gradient shadow -- repainted
  // every time the camera moves, which is what makes zooming stutter.
  const hasBuildings = (mapData.buildings || []).length > 0;
  const huge = drawRoads.length > 1500;
  if (hasBuildings || huge) props = [];
  else buildOccupancyAndTrees(b.minX, b.minY, b.maxX, b.maxY);

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
      // The server reports one light per APPROACH, keyed by the directed road
      // id. A drawn road carries both directions of a two-way street: arriving
      // at this junction along it is the forward road when the junction is its
      // "to" end, and the "_r" twin when it is the "from" end.
      const signalId = r.to === n.id ? r.id : r.id + "_r";
      signalHeads.push({
        baseId: r.id,
        signalId,                                 // -> lightState key
        nodeId: n.id,
        ax: a.x, ay: a.y, bx: b.x, by: b.y,       // approach axis, for the setback
        sx: b.x - ux * back, sy: b.y - uy * back, // fallback stop line
        ux, uy,
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
  const treeW = Math.max(6, 9 * US * 4);   // world-unit canopy width
  // A tree needs its trunk clear of the asphalt by half the road plus half its
  // own canopy. The stamping below marks every cell within ONE RING of a road,
  // so any cell left free is already at least occCell away from one -- size the
  // cell to exactly that clearance and "is this cell free" becomes the whole
  // test. The previous rule (a fixed 14 m cell, and the four neighbours free as
  // well) demanded three clear cells in a row, which on a 240 m block with 30
  // unit-wide roads is wider than the gap between them: it produced no scenery
  // at all on precisely the hand-drawn maps the scatter exists for.
  occCell = Math.max(12, roadW * 0.5 + treeW * 0.5 + 4);
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
  for (let y = minY; y < maxY && props.length < maxTrees; y += spacing) {
    for (let x = minX; x < maxX && props.length < maxTrees; x += spacing) {
      if (rng() < 0.35) continue;                       // thin it out
      const jx = x + (rng() - 0.5) * spacing * 0.7;
      const jy = y + (rng() - 0.5) * spacing * 0.7;
      const cx = cellX(jx), cy = cellY(jy);
      if (cx < 1 || cy < 1 || cx >= occCols - 1 || cy >= occRows - 1) continue;
      if (occ[cy * occCols + cx]) continue;             // road or building here
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
        const isRev = r.id.endsWith("_r");
        if (isRev) continue;  // Base road handles asphalt and markings for full street

        const a = nodeById[r.from], b = nodeById[r.to];
        if (!a || !b || !segVisible(a, b, cull)) continue;
        const x1 = wx(a.x), y1 = wy(a.y), x2 = wx(b.x), y2 = wy(b.y);
        const totalW = (roadWOf[r.id] || roadW);
        g.strokeStyle = C.asphalt;
        g.lineWidth = totalW * T.s;
        g.beginPath(); g.moveTo(x1, y1); g.lineTo(x2, y2); g.stroke();

        if (!laneDetail) continue;
        const len = Math.hypot(x2 - x1, y2 - y1) || 1;
        const px = -(y2 - y1) / len, py = (x2 - x1) / len;

        const revRoad = roadById[r.id + "_r"];
        const twoWay = !r.oneway && !!revRoad;

        const mark = (o, dashed, strokeStyle = C.dash) => {
          g.strokeStyle = strokeStyle;
          g.setLineDash(dashed ? [9 * US * T.s, 13 * US * T.s] : []);
          g.beginPath();
          g.moveTo(x1 + px * o * T.s, y1 + py * o * T.s);
          g.lineTo(x2 + px * o * T.s, y2 + py * o * T.s);
          g.stroke();
        };

        g.lineWidth = Math.max(1, 1.4 * US * T.s);

        if (twoWay) {
          // Centre line: solid line separating the two opposing directions
          mark(0, false, C.dash);
          // Lane dividers within each direction
          const halfW = totalW * 0.5;
          const lanesFwd = Math.max(1, r.lanes || 1);
          const laneWFwd = halfW / lanesFwd;
          for (let i = 1; i < lanesFwd; i++) {
            mark(i * laneWFwd, true);
          }
          const lanesRev = Math.max(1, revRoad.lanes || lanesFwd);
          const laneWRev = halfW / lanesRev;
          for (let i = 1; i < lanesRev; i++) {
            mark(-i * laneWRev, true);
          }
        } else {
          // 1-way road: lane dividers between adjacent lanes
          const totalLanes = Math.max(1, r.lanes || 1);
          const laneW = totalW / totalLanes;
          const halfW = totalW * 0.5;
          for (let i = 1; i < totalLanes; i++) {
            mark(-halfW + i * laneW, true);
          }
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
let lightState = {};
let lightSetback = {};   // road id -> metres the stop line sits before the junction
// The vehicle whose route is highlighted, and the road ids of that route.
// Fetched once per selection from /api/vehicle/detail rather than carried in
// /api/state: a route can be 60+ roads, and multiplying that by every vehicle
// on screen would dwarf the rest of the payload for data nobody looks at.
let routeVehicleId = null;
let routeRoadIds = [];            // nodeId -> 0|1|2
let roadState = {};             // roadId -> { congestion, closed }
let simOnline = false;

// Snapshot interpolation. Vehicles are drawn ~one poll interval in the PAST,
// linearly blended between the two most recent /api/state samples, so motion is
// continuous whatever the poll rate or the tick rate. The obvious alternative --
// easing the drawn position toward the newest sample -- is what this replaces:
// it lags by its own time constant, never reaches the target, and pulses once
// per poll, which is exactly what "jerky" looks like.
let snapPrev = null, snapCur = null;   // { t, byId: Map(id -> placed vehicle) }
let pollInterval = 150;                // ms, recomputed after each poll
let interpDelay = 210;                 // ms, ~1.4x the poll interval

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
  for (const l of s.lights || []) {
    lightState[l.id] = l.state;
    if (l.d !== undefined) lightSetback[l.id] = l.d;
  }
  roadState = {};
  for (const r of s.roads || []) roadState[r.id] = r;

  const byId = new Map();
  for (const v of s.vehicles || []) {
    const p = placeOnRoad(v.road, v.pos, v.lane || 0, v.lat || 0);
    if (!p) continue;
    byId.set(v.id, { id: v.id, type: v.type, road: v.road, speed: v.speed || 0,
                     x: p.x, y: p.y, h: p.h });
  }
  snapPrev = snapCur;
  snapCur = { t: performance.now(), byId };

  // Drop the steering state of everyone who has left, or the maps grow without
  // bound over a long run.
  for (const id of displayHeading.keys()) if (!byId.has(id)) displayHeading.delete(id);
  for (const id of interpPool.keys()) if (!byId.has(id)) interpPool.delete(id);

  // The highlighted vehicle finished its trip or was removed: drop the path
  // rather than leave a route glowing under traffic that is no longer there.
  if (routeVehicleId && !byId.has(routeVehicleId)) clearRouteHighlight();
}

// Blend the two newest snapshots at (now - interpDelay). Both the output
// objects and the array are REUSED between frames: at 60 fps across a
// four-figure fleet, allocating fresh ones every frame is enough garbage to
// show up as periodic stalls.
const interpPool = new Map();       // id -> reusable render object
const interpOut = [];

function lerpAngle(a, b, t) {
  let d = b - a;
  while (d > Math.PI) d -= Math.PI * 2;
  while (d < -Math.PI) d += Math.PI * 2;
  return a + d * t;
}

function interpolatedVehicles() {
  interpOut.length = 0;
  if (!snapCur) return interpOut;
  const blend = snapPrev && snapCur.t > snapPrev.t;
  const at = performance.now() - interpDelay;
  let t = blend ? (at - snapPrev.t) / (snapCur.t - snapPrev.t) : 1;
  t = Math.max(0, Math.min(1.25, t));    // a little extrapolation beats stalling
  const tc = Math.min(1, t);
  // How far a vehicle can legitimately travel between two samples. Anything
  // beyond it is a reroute, a rescue or a respawn -- snap, rather than slide
  // the sprite across town.
  const jump = 150 * Math.max(1, simState ? simState.speed || 1 : 1);
  for (const [id, cur] of snapCur.byId) {
    const prev = blend ? snapPrev.byId.get(id) : null;
    if (!prev || Math.hypot(cur.x - prev.x, cur.y - prev.y) > jump) {
      displayHeading.delete(id);        // don't animate a turn across a teleport
      interpOut.push(cur);
      continue;
    }
    let o = interpPool.get(id);
    if (!o) { o = {}; interpPool.set(id, o); }
    o.id = id;
    o.type = cur.type;
    o.road = cur.road;
    o.speed = cur.speed;
    o.x = prev.x + (cur.x - prev.x) * t;
    o.y = prev.y + (cur.y - prev.y) * t;
    o.h = lerpAngle(prev.h, cur.h, tc);
    interpOut.push(o);
  }
  return interpOut;
}

// Displayed heading per vehicle, eased toward the target at a capped angular
// speed. Interpolating the heading alone still snaps at a corner, because the
// two samples either side of it are 90 degrees apart; capping the rate is what
// makes the turn read as steering.
const displayHeading = new Map();

function smoothHeading(id, target) {
  let h = displayHeading.get(id);
  if (h === undefined) { displayHeading.set(id, target); return target; }
  let d = target - h;
  while (d > Math.PI) d -= Math.PI * 2;
  while (d < -Math.PI) d += Math.PI * 2;
  // ~7 rad/s at 60 fps, and quicker when the simulation itself is sped up.
  const maxStep = 0.12 * Math.max(1, simState ? simState.speed || 1 : 1);
  h += Math.abs(d) <= maxStep ? d : Math.sign(d) * maxStep;
  if (h > Math.PI) h -= Math.PI * 2;
  if (h < -Math.PI) h += Math.PI * 2;
  displayHeading.set(id, h);
  return h;
}

// The drawn width of one lane on a given road, so a sprite can be capped to it
// and never overhang the asphalt.
function laneWidthOf(roadId) {
  const r = roadById[roadId];
  if (!r) return roadW / 4;
  const base = roadId.endsWith("_r") ? roadId.slice(0, -2) : roadId;
  return (roadWOf[base] || roadW) * 0.5 / Math.max(1, r.lanes || 1);
}

function drawVehicles() {
  const vs = interpolatedVehicles();
  if (!vs.length) return;
  const rMinX = -T.tx / T.s, rMaxX = (canvas.width - T.tx) / T.s;
  const rMinY = -T.ty / T.s, rMaxY = (canvas.height - T.ty) / T.s;

  // Painter's order so a sprite overlapping another resolves consistently --
  // pointless once everything is a few pixels across.
  if (19 * VS * T.s >= 4) vs.sort((a, b) => a.y - b.y);

  for (const e of vs) {
    if (e.x < rMinX - 40 || e.x > rMaxX + 40 || e.y < rMinY - 40 || e.y > rMaxY + 40) continue;

    const lenM = VEH_LEN[e.type] || 4.5;
    // Sprite size depends on physical vehicle length and map zoom scale,
    // ensuring size never changes when switching between roads of different lane counts.
    const w = Math.max(3, lenM * VS * 1.5 * T.s);
    const h = w * 0.5;
    const sx = wx(e.x), sy = wy(e.y);
    const heading = smoothHeading(e.id, e.h);

    const spr = e.type === "car"
      ? Sprites.pick("car", 4, hashInt(e.id))
      : Sprites.get(e.type + "-1");

    ctx.save();
    ctx.translate(sx, sy);
    ctx.rotate(heading);
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
    const st = lightState[h.signalId];
    ctx.fillStyle = st === undefined ? C.signalHead : LIGHT_COLOR[st] || C.signalHead;
    const off = (roadWOf[h.baseId] || roadW) / 2 + 2.5 * US;
    // The stop line is where the SERVER says it is, so the head sits exactly
    // where vehicles are actually braking. Falls back to the geometric guess
    // until the first state arrives.
    const d = lightSetback[h.signalId];
    let sx = h.sx, sy = h.sy;
    if (d !== undefined) {
      const back = Math.min(d * US, Math.hypot(h.bx - h.ax, h.by - h.ay) * 0.5);
      sx = h.bx - h.ux * back;
      sy = h.by - h.uy * back;
    }
    ctx.beginPath();
    ctx.arc(wx(sx + h.px * off), wy(sy + h.py * off), dot, 0, Math.PI * 2);
    ctx.fill();
  }
}

// Roads worth drawing status for, merged to one entry per PHYSICAL street and
// resolved to endpoints. Rebuilt once per poll, not once per frame: the state
// changes four to twenty times a second and the screen redraws sixty, so doing
// this per frame was re-deriving the same answer fifteen times over -- with a
// fresh key array allocated each time.
let liveRoads = [];

// Below this the tint is a wash over the road and says nothing;
// reference project uses the same threshold.
const CONGESTION_FLOOR = 0.12;

const CONG_COLORS = ["#5d9b4f", "#a5b23f", "#d9a33b", "#d96f33", "#c23b3b"];

function congestionColor(c) {
  const idx = Math.min(CONG_COLORS.length - 1, Math.floor(c * CONG_COLORS.length));
  return CONG_COLORS[idx];
}

function rebuildLiveRoads() {
  const list = [];
  for (const id of Object.keys(roadState)) {
    const st = roadState[id];
    const c = st.congestion || 0;
    if (c <= CONGESTION_FLOOR && !st.closed) continue;
    const isRev = id.endsWith("_r");
    const base = isRev ? id.slice(0, -2) : id;
    const r = roadById[base] || roadById[id];
    if (!r) continue;

    const fromId = isRev ? r.to : r.from;
    const toId = isRev ? r.from : r.to;
    const a = nodeById[fromId], b = nodeById[toId];
    if (!a || !b) continue;

    list.push({
      id,
      base,
      isRev,
      isOneway: !!r.oneway,
      ax: a.x, ay: a.y,
      bx: b.x, by: b.y,
      c,
      closed: !!st.closed
    });
  }
  liveRoads = list;
}

// Congested / closed roads, drawn over the asphalt per direction (50% width each for 2-way roads).
function drawRoadStatus() {
  if (!liveRoads.length) return;
  const rMinX = -T.tx / T.s, rMaxX = (canvas.width - T.tx) / T.s;
  const rMinY = -T.ty / T.s, rMaxY = (canvas.height - T.ty) / T.s;
  ctx.lineCap = "round";
  for (const e of liveRoads) {
    if (Math.max(e.ax, e.bx) < rMinX || Math.min(e.ax, e.bx) > rMaxX ||
        Math.max(e.ay, e.by) < rMinY || Math.min(e.ay, e.by) > rMaxY) continue;

    const dx = e.bx - e.ax;
    const dy = e.by - e.ay;
    const len = Math.hypot(dx, dy);

    const totalWWorld = roadWOf[e.base] || roadW;
    let startX = e.ax, startY = e.ay;
    let endX = e.bx, endY = e.by;
    let strokeW = totalWWorld * T.s * 0.5;

    if (!e.isOneway && len > 1e-6) {
      // Offset perpendicular to the right side of travel direction (matching vehicle placeOnRoad math)
      const nx = -dy / len;
      const ny = dx / len;
      const offset = totalWWorld * 0.25;
      startX += nx * offset;
      startY += ny * offset;
      endX += nx * offset;
      endY += ny * offset;
    } else if (e.isOneway) {
      strokeW = totalWWorld * T.s * 0.8;
    }

    if (e.c > CONGESTION_FLOOR) {
      ctx.save();
      ctx.globalAlpha = 0.55;
      ctx.strokeStyle = congestionColor(e.c);
      ctx.lineWidth = Math.max(1.5, strokeW);
      ctx.beginPath();
      ctx.moveTo(wx(startX), wy(startY));
      ctx.lineTo(wx(endX), wy(endY));
      ctx.stroke();
      ctx.restore();
    }

    if (e.closed) {
      ctx.save();
      ctx.globalAlpha = 0.75;
      ctx.strokeStyle = "rgba(239,68,68,.85)";
      ctx.lineWidth = Math.max(2.0, strokeW);
      ctx.beginPath();
      ctx.moveTo(wx(startX), wy(startY));
      ctx.lineTo(wx(endX), wy(endY));
      ctx.stroke();
      ctx.restore();

      drawBarrier(wx((startX + endX) / 2), wy((startY + endY) / 2),
                  Math.atan2(dy, dx));
    }
  }
}

// The selected vehicle's remaining route, drawn under the traffic so it reads
// as a path on the map rather than something floating above it.
// Two passes per road, like the reference: a wide translucent halo so the path
// is findable at a glance, and a narrow bright core so it stays readable where
// the route doubles back on itself or runs beside another highlighted road.
// One medium stroke does neither job well.
function drawRouteHighlight() {
  if (!routeRoadIds.length) return;
  ctx.lineCap = "round";
  ctx.strokeStyle = "#3f8fe0";
  const halo = Math.max(6, 14 * US * T.s);
  const core = Math.max(2, 5 * US * T.s);
  for (const pass of [{ w: halo, a: 0.30 }, { w: core, a: 0.90 }]) {
    ctx.globalAlpha = pass.a;
    ctx.lineWidth = pass.w;
    ctx.beginPath();
    for (const id of routeRoadIds) {
      const r = roadById[id];
      if (!r) continue;
      const a = nodeById[r.from], b = nodeById[r.to];
      if (!a || !b) continue;
      ctx.moveTo(wx(a.x), wy(a.y));
      ctx.lineTo(wx(b.x), wy(b.y));
    }
    ctx.stroke();
  }
  ctx.globalAlpha = 1;
}

// ---- render loop + fps meter ----
let fpsFrames = 0, fpsWorst = 0, fpsWindowStart = 0, lastFrameTs = 0;
// ---------------------------------------------------------------------------
// Map-editing feedback and closed-road markers
// ---------------------------------------------------------------------------

// A barrier across a closed road, drawn at its midpoint. Recolouring the road
// is not enough on a dense map: at district zoom a red line among orange
// congestion reads as "busy", not "shut".
function drawBarrier(x, y, angle) {
  const s = Math.max(0.35, T.s);
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(angle);
  ctx.fillStyle = "#f2efe2";
  ctx.fillRect(-7 * s, -2.6 * s, 14 * s, 5.2 * s);
  ctx.fillStyle = "rgba(239,68,68,.95)";
  for (let i = -7; i < 7; i += 3.5) ctx.fillRect(i * s, -2.6 * s, 1.8 * s, 5.2 * s);
  ctx.restore();
}

function drawZoneLabels() {
  const zones = (mapData && mapData.zones) || [];
  if (!zones.length) return;
  const size = Math.max(9, 13 * T.s);
  ctx.font = `700 ${size}px system-ui, sans-serif`;
  ctx.textAlign = "center";
  ctx.textBaseline = "top";
  ctx.lineWidth = Math.max(2, size * 0.28);
  for (const z of zones) {
    if (!z.name) continue;
    const sx = wx(z.x + z.w / 2), sy = wy(z.y + z.h) + 14 * T.s;
    ctx.strokeStyle = "rgba(255,255,255,.85)";   // halo, so it reads over roads
    ctx.strokeText(z.name, sx, sy);
    ctx.fillStyle = "rgba(30,41,59,.9)";
    ctx.fillText(z.name, sx, sy);
  }
}

const ENDPOINT_COLORS = { junction: "#4f8fd0", road: "#e8a33d", free: "#5cb85c" };

function drawEndpointMarker(e) {
  const sx = wx(e.x), sy = wy(e.y);
  ctx.beginPath();
  ctx.arc(sx, sy, 7, 0, Math.PI * 2);
  ctx.fillStyle = ENDPOINT_COLORS[e.kind] || "#888";
  ctx.globalAlpha = 0.85;
  ctx.fill();
  ctx.globalAlpha = 1;
  ctx.lineWidth = 2;
  ctx.strokeStyle = "#ffffff";
  ctx.stroke();
  if (!e.label) return;
  ctx.font = "bold 12px system-ui, sans-serif";
  ctx.textAlign = "left";
  ctx.textBaseline = "alphabetic";
  ctx.lineWidth = 3;
  ctx.strokeStyle = "rgba(0,0,0,.75)";
  ctx.strokeText(e.label, sx + 11, sy - 8);
  ctx.fillStyle = "#ffffff";
  ctx.fillText(e.label, sx + 11, sy - 8);
}

// Turn a world point into the endpoint the server will resolve it to, using the
// same priority the server uses: junction, then a point on a road (which splits
// it), then open ground (which creates a junction). Showing the wrong guess
// would be worse than showing nothing, so this mirrors pickAt().
function previewEndpoint(p) {
  const hit = pickAt(p.x, p.y);
  if (hit && hit.kind === "junction") {
    return { kind: "junction", x: hit.node.x, y: hit.node.y,
             label: hit.node.name || hit.id };
  }
  if (hit && hit.kind === "road") {
    return { kind: "road", x: p.x, y: p.y, label: "split " + hit.id };
  }
  return { kind: "free", x: p.x, y: p.y, label: "new junction" };
}

function drawAddRoadPreview() {
  if (!UI.isAddingRoad() || !mapData) return;
  const first = UI.pendingRoadStart();
  const cur = hoverWorld ? previewEndpoint(hoverWorld) : null;
  if (!first && !cur) return;

  // The stored first endpoint has coordinates only when it was free ground or
  // a road split; a junction was stored by id, so look its position back up.
  let a = null;
  if (first) {
    a = first.kind === "junction"
      ? (() => { const n = nodeById[first.id]; return n && { kind: "junction", x: n.x, y: n.y, label: n.name || first.id }; })()
      : { kind: first.kind, x: first.x, y: first.y,
          label: first.kind === "road" ? "split here" : "new junction" };
  }

  if (a && cur) {
    ctx.save();
    ctx.beginPath();
    ctx.setLineDash([8, 6]);
    ctx.moveTo(wx(a.x), wy(a.y));
    ctx.lineTo(wx(cur.x), wy(cur.y));
    ctx.lineWidth = 2.5;
    ctx.strokeStyle = "#ffffff";
    ctx.globalAlpha = 0.8;
    ctx.stroke();
    ctx.restore();
  }
  if (a) drawEndpointMarker(a);
  if (cur) drawEndpointMarker(cur);
}

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
    drawRouteHighlight();
    drawRoadStatus();
    drawSignals();
    drawVehicles();
    drawZoneLabels();
    drawAddRoadPreview();
  }
  requestAnimationFrame(render);
}

// ---------------------------------------------------------------------------
// Simulation polling
// ---------------------------------------------------------------------------
let pollTimer = null;

// Self-scheduling rather than setInterval: poll() is async, and an interval
// would start the next fetch while the previous one is still out, so two
// replies could land out of order and hand the interpolator a snapshot older
// than the one it already has.
async function poll() {
  const s = await loadState();
  if (s) {
    if (!simOnline) {
      simOnline = true;
      UI.log("Simulation connected");
    }
    applyState(s);
    rebuildLiveRoads();
  } else if (simOnline) {
    simOnline = false;
    simState = null;
    UI.log("Simulation offline");
  }
  // Keep the highlighted route in step with the vehicle that owns it.
  if (routeVehicleId) refreshRoute();
  UI.update(simState);

  // A faster simulation covers more ground between two samples, so sample more
  // often: 150 ms at 1x down to 45 ms at 10x, keeping the distance travelled
  // per sample roughly fixed. The render delay follows it.
  const spd = Math.max(1, simState ? simState.speed || 1 : 1);
  pollInterval = Math.max(45, Math.min(150, Math.round(150 / spd)));
  interpDelay = Math.max(90, pollInterval * 1.4);
  if (pollTimer !== null) clearTimeout(pollTimer);
  pollTimer = setTimeout(poll, pollInterval);
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
  // Pick against the DRAWN positions, not the newest snapshot: the sprite the
  // user aimed at is the interpolated one, and at speed the two are metres
  // apart.
  for (const e of interpolatedVehicles()) {
    const d = Math.hypot(e.x - wxp, e.y - wyp);
    if (d < bestD) { bestD = d; best = { kind: "vehicle", id: e.id, e }; }
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
      ["Reroutes", v.recalcs ?? 0],
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

  if (!hit) { clearRouteHighlight(); UI.hideSelection(); return; }
  const [title, rows] = describe(hit);
  UI.showSelection(title, rows);

  if (hit.kind === "vehicle") selectVehicleRoute(hit.id);
  else clearRouteHighlight();
}

// Ask the server for the selected vehicle's remaining route and highlight it.
// Fire-and-forget: if the vehicle arrives before the reply lands, the ids
// simply no longer match any road and nothing is drawn.
async function selectVehicleRoute(id) {
  routeVehicleId = id;
  routeRoadIds = [];
  lastRouteFetch = -Infinity;               // fetch immediately
  await refreshRoute();
}

// The route shortens as the vehicle drives, and changes outright when an event
// makes it replan. Fetching once at click time leaves the highlight showing
// roads already driven, and pointing the wrong way entirely after a reroute --
// so re-ask while the vehicle stays selected. Throttled, because this is the
// one call that does take the simulation lock.
// -Infinity, not 0: performance.now() is measured from page load, so a plain 0
// makes the throttle swallow the very first fetch when a vehicle is clicked in
// the first 400 ms after the page appears.
let lastRouteFetch = -Infinity;
const ROUTE_REFRESH_MS = 400;

async function refreshRoute() {
  const id = routeVehicleId;
  if (!id) return;
  const now = performance.now();
  if (now - lastRouteFetch < ROUTE_REFRESH_MS) return;
  lastRouteFetch = now;
  try {
    const d = await Sim.vehicleDetail(id);
    if (routeVehicleId !== id) return;      // selection changed while waiting
    routeRoadIds = Array.isArray(d?.route) ? d.route : [];
  } catch (err) {
    // Keep the previous route rather than blinking it away on one bad poll --
    // but say something. A silent catch here is what hid a missing `Sim`
    // import: every call threw ReferenceError, was swallowed, and the route
    // highlight simply never appeared with nothing in the console to show for
    // it. A dropped network poll is expected; a bug is not, and the two look
    // identical from in here.
    console.warn("route refresh failed", err);
  }
}

function clearRouteHighlight() {
  routeVehicleId = null;
  routeRoadIds = [];
}

// Remove a client-side prop by index (used by "Delete Decoration").
function removePropAt(i) {
  if (i < 0 || i >= props.length) return false;
  props.splice(i, 1);
  staticDirty = true;
  return true;
}

// Re-fetch the map after a server-side edit and rebuild the static layer.
// Deliberately NOT prepareMap(): an edit is a change to the city you are
// already looking at, so the camera stays put and client-placed scenery is
// left alone. Loading a different map is what calls prepareMap().
async function reloadMap() {
  mapData = await loadMap();
  rebuildDerived();
}

// A DIFFERENT city (Files -> Open, or an OpenStreetMap import). Everything
// derived from the old one is wrong now, including the scenery scattered into
// its gaps and the camera framing its extent, so this is the full path.
async function reloadCity() {
  mapData = await loadMap();
  prepareMap();
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

// ---------------------------------------------------------------------------
// OpenStreetMap area picker
// ---------------------------------------------------------------------------
function initOsmPicker() {
  const overlay = document.getElementById("osm-overlay");
  const cv = document.getElementById("osm-canvas");
  const status = document.getElementById("osm-status");
  const sizeEl = document.getElementById("osm-size");
  const lockBtn = document.getElementById("osm-lock");
  if (!overlay || !cv) return;

  const picker = new OsmPicker(cv);
  let busy = false;

  const say = (msg, isError) => {
    if (!msg) { status.classList.add("hidden"); return; }
    status.textContent = msg;
    status.classList.toggle("error", !!isError);
    status.classList.remove("hidden");
  };

  const fit = () => {
    // The picker canvas is laid out by flexbox, so its backing store has to be
    // resized to match or the tiles come out stretched.
    cv.width = cv.clientWidth;
    cv.height = cv.clientHeight;
    picker.draw();
  };

  picker.onChange = (sel, span) => {
    if (!sel) {
      sizeEl.textContent = "";
      sizeEl.classList.remove("too-big");
      lockBtn.disabled = true;
      return;
    }
    const area = span.w * span.h;
    sizeEl.textContent = `${span.w.toFixed(1)} x ${span.h.toFixed(1)} km`;
    // Warn before the download, not after: a 10x10 km extract is minutes of
    // waiting and a graph too big to simulate smoothly.
    sizeEl.classList.toggle("too-big", area > 25);
    lockBtn.disabled = busy || area > 25;
  };

  const open = () => {
    overlay.classList.remove("hidden");
    picker.active = true;
    UI.suspendEventLog();       // it floats above the overlay otherwise
    say(null);
    requestAnimationFrame(fit);
  };
  const close = () => {
    picker.active = false;
    overlay.classList.add("hidden");
    UI.restoreEventLog();
  };

  document.getElementById("menu-osm")?.addEventListener("click", open);
  document.getElementById("osm-cancel").addEventListener("click", close);
  document.getElementById("osm-clear").addEventListener("click", () => picker.clearSelection());
  window.addEventListener("resize", () => { if (picker.active) fit(); });

  lockBtn.addEventListener("click", async () => {
    if (!picker.sel || busy) return;
    busy = true;
    lockBtn.disabled = true;
    const name = (document.getElementById("osm-name").value || "osm-import").trim();
    try {
      const r = await importArea(picker.sel, name, (m) => say(m));
      // The backend has already swapped the running map, so the canvas has to
      // throw away everything it derived from the old one -- scenery and camera
      // included, which is what separates this from an in-place edit.
      await reloadCity();
      close();
      UI.log(`Imported ${r.roads} roads and ${r.signals} traffic signals ` +
             `from OpenStreetMap as "${name}"`);
      UI.toast(r.warning ? `Imported, but: ${r.warning}` : "Map imported");
    } catch (err) {
      say(String(err.message || err), true);
    } finally {
      busy = false;
      lockBtn.disabled = !picker.sel;
    }
  });
}

async function boot() {
  initInput();
  initOsmPicker();
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
      reloadCity,
      getState: () => simState,
      getMap: () => mapData,
      getDrawRoads: () => drawRoads,
      getPropCount: () => props.length,
    });

    await poll();          // schedules its own next run
    requestAnimationFrame(render);
  } catch (err) {
    console.error(err);
    showError(String(err.message || err));
    UI.toast("Failed to load the map");
  }
}

const ready = boot();

// Read-only window into the renderer for web_frontend/tests/. The tests drive
// this module for real -- real map, real /api/state, real draw calls -- so they
// need to see what it decided, not a copy of the logic. Nothing here is used by
// the page itself.
export const __debug = {
  ready,
  get T() { return T; },
  get fitScale() { return fitScale; },
  get US() { return US; },
  get VS() { return VS; },
  get mapData() { return mapData; },
  get roadById() { return roadById; },
  get nodeById() { return nodeById; },
  get drawRoads() { return drawRoads; },
  get props() { return props; },
  get simState() { return simState; },
  get routeRoadIds() { return routeRoadIds; },
  get routeVehicleId() { return routeVehicleId; },
  get liveRoads() { return liveRoads; },
  get pollInterval() { return pollInterval; },
  get interpDelay() { return interpDelay; },
  snapTimes: () => [snapPrev && snapPrev.t, snapCur && snapCur.t],
  stopPolling() {
    if (pollTimer !== null) { clearTimeout(pollTimer); pollTimer = null; }
  },
  // A copy on purpose: the renderer reuses one pooled object per vehicle, so
  // the live array is only valid until the next call. Tests hold on to results.
  drawnVehicles: () => interpolatedVehicles().map((v) => ({ ...v })),
  placeOnRoad,
  selectVehicleRoute,
  clearRouteHighlight,
  reloadMap,
  reloadCity,
  prepareMap,
  fitView,
  poll,
  onMapClick,
};
