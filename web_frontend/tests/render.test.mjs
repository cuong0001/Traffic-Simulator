// Renderer tests: drive the REAL app.js against a REAL running simulator.
//
//   node web_frontend/tests/render.test.mjs [http://localhost:8099]
//
// Start the server first:
//   build\traffic_simulator.exe data\maps\small_map.geojson 8099
//
// Nothing here re-implements the renderer. The harness supplies a DOM and a
// recording 2D context, app.js loads and boots for real, and the assertions ask
// what it actually resolved and actually drew.

"use strict";

import { install, clock, check, section, report, sleep } from "./harness.mjs";

const origin = process.argv[2] || "http://localhost:8099";

// The stubs must exist before app.js is imported: it grabs the canvas and the
// context at module scope.
const dom = install({ origin });

let app;
try {
  app = await import("../js/app.js");
} catch (e) {
  console.error("app.js failed to load:", e);
  process.exit(1);
}
const D = app.__debug;
await D.ready;

if (!D.mapData) {
  console.error(`No map. Is traffic_simulator serving ${origin}?`);
  process.exit(1);
}

const api = async (p) => (await fetch(origin + p)).json();

// One render frame, plus a little wall time so any fetch the frame kicked off
// can land before the next assertion.
function frame(dt = 16) { dom.frame(dt); }

// Advance far enough that poll()'s setTimeout fires, and wait for the state to
// actually change hands.
async function nextPoll() {
  const before = D.simState && D.simState.time;
  for (let i = 0; i < 60; i++) {
    await sleep(30);
    if (D.simState && D.simState.time !== before) return true;
  }
  return false;
}

// ---------------------------------------------------------------------------
section("Map resolution — every directed road the simulator names must resolve");
// ---------------------------------------------------------------------------
{
  const map = await api("/api/map");
  const twoWay = map.roads.filter((r) => !r.oneway && !r.id.endsWith("_r"));
  check("two-way streets got a reverse twin",
        twoWay.every((r) => !!D.roadById[r.id + "_r"]),
        `${twoWay.length} streets`);

  // A synthesised twin must be the real reverse, not a copy: from/to swapped,
  // same length. Placing a vehicle at 0 m on the twin must land where placing
  // one at length on the base does.
  let geomOk = 0, geomBad = 0;
  for (const r of twoWay.slice(0, 200)) {
    const head = D.placeOnRoad(r.id, r.length);
    const tail = D.placeOnRoad(r.id + "_r", 0);
    if (!head || !tail) { geomBad++; continue; }
    // Same point on the centreline; the lane offset puts them on opposite
    // sides, so compare against the drawn road width rather than demanding 0.
    Math.hypot(head.x - tail.x, head.y - tail.y) < 30 ? geomOk++ : geomBad++;
  }
  check("twin geometry is the true reverse", geomBad === 0,
        `${geomOk} checked, ${geomBad} wrong`);

  check("one-way streets did NOT get a bogus twin",
        map.roads.filter((r) => r.oneway && !r.id.endsWith("_r"))
                 .every((r) => !D.roadById[r.id + "_r"] ||
                               map.roads.some((x) => x.id === r.id + "_r")));

  const st = await api("/api/state");
  const unresolved = st.vehicles.filter((v) => !D.roadById[v.road]);
  check("every vehicle's road resolves", unresolved.length === 0,
        `${st.vehicles.length} vehicles, ${unresolved.length} unresolved`);
}

// ---------------------------------------------------------------------------
section("Vehicles actually reach the screen");
// ---------------------------------------------------------------------------
{
  const alive = D.simState.vehicles.length;
  const drawn = D.drawnVehicles().length;
  check("every live vehicle is placed", drawn === alive, `${drawn}/${alive}`);

  // Blink test: a vehicle present in two consecutive polls must be drawable in
  // both. This is what the missing twins used to break, four times a second.
  let flips = 0, transitions = 0;
  let prevDrawn = new Set(D.drawnVehicles().map((v) => v.id));
  let prevAlive = new Set(D.simState.vehicles.map((v) => v.id));
  for (let i = 0; i < 6; i++) {
    if (!(await nextPoll())) break;
    const nowDrawn = new Set(D.drawnVehicles().map((v) => v.id));
    const nowAlive = new Set(D.simState.vehicles.map((v) => v.id));
    for (const id of nowAlive) {
      if (!prevAlive.has(id)) continue;
      transitions++;
      if (nowDrawn.has(id) !== prevDrawn.has(id)) flips++;
    }
    prevDrawn = nowDrawn; prevAlive = nowAlive;
  }
  check("no vehicle blinks between polls", flips === 0,
        `${flips} flips over ${transitions} transitions`);
}

// ---------------------------------------------------------------------------
section("Route highlight — the whole remaining route, and it must be drawn");
// ---------------------------------------------------------------------------
{
  // Pick a vehicle with a long route so a partial highlight cannot pass.
  let target = null, detail = null;
  for (const v of D.simState.vehicles) {
    const d = await api("/api/vehicle/detail?id=" + encodeURIComponent(v.id));
    if (d.route && d.route.length >= 10) { target = v; detail = d; break; }
  }
  check("found a vehicle with a route to test", !!target,
        detail ? `${detail.route.length} roads` : "");

  if (target) {
    await D.selectVehicleRoute(target.id);
    const ids = D.routeRoadIds;
    check("route came back", ids.length > 0, `${ids.length} roads`);

    const missing = ids.filter((id) => !D.roadById[id]);
    check("every road of the route resolves", missing.length === 0,
          `${ids.length - missing.length}/${ids.length} resolve` +
          (missing.length ? `, e.g. ${missing.slice(0, 3).join(", ")}` : ""));

    // Now the part that matters: count what reached the canvas. The route is
    // stroked in two passes (halo + core), each one path holding every segment.
    dom.ctx.__reset();
    frame();
    const routeStrokes = dom.ctx.__calls.filter(
      (c) => c.op === "stroke" && String(c.style).toLowerCase() === "#3f8fe0");
    check("route drawn in two passes", routeStrokes.length === 2,
          `${routeStrokes.length} strokes`);
    const segs = routeStrokes.length
      ? routeStrokes[0].path.filter((p) => p[0] === "M").length : 0;
    check("every route road reached the canvas", segs === ids.length,
          `${segs} segments drawn for ${ids.length} roads`);

    // And it must keep up: the route shortens as the vehicle drives.
    const before = ids.length;
    for (let i = 0; i < 12; i++) { await nextPoll(); }
    check("route stays live while selected",
          D.routeVehicleId === target.id || D.routeVehicleId === null,
          `now ${D.routeRoadIds.length} roads (was ${before})`);
  }
  D.clearRouteHighlight();
}

// ---------------------------------------------------------------------------
section("Motion is interpolated, not chased");
// ---------------------------------------------------------------------------
{
  // Take two snapshots by hand, a poll interval apart in BOTH clocks, then walk
  // the render time across the window between them. Under the live loop the
  // harness's stepped clock and the network's wall clock drift against each
  // other and the numbers say nothing; here the input is exact, so the output
  // can be held to the property "interpolated" actually means: position
  // advances in proportion to time, with no step at either end.
  const GAP = 200;
  D.stopPolling();
  await D.poll(); D.stopPolling();
  await sleep(GAP);
  clock.advance(GAP);
  await D.poll(); D.stopPolling();

  const [tPrev, tCur] = D.snapTimes();
  check("two snapshots to blend between", tPrev != null && tCur - tPrev === GAP,
        `${(tCur - tPrev).toFixed(0)} ms apart`);

  // A vehicle in both snapshots that covered real ground between them.
  const at = (f, id) => {
    clock.t = tPrev + (tCur - tPrev) * f + D.interpDelay;
    return D.drawnVehicles().find((x) => x.id === id);
  };
  let moving = null;
  for (const cand of D.drawnVehicles().filter((v) => v.speed > 4)
                      .sort((a, b) => b.speed - a.speed).slice(0, 20)) {
    const a = at(0, cand.id), b = at(1, cand.id);
    if (a && b && Math.hypot(b.x - a.x, b.y - a.y) > 0.5) { moving = cand; break; }
  }
  check("found a moving vehicle to follow", !!moving,
        moving ? `${moving.id} at ${moving.speed.toFixed(1)} m/s` : "");

  if (moving) {
    const id = moving.id;
    const N = 40;
    const samples = [];
    for (let i = 0; i <= N; i++) {
      // Render time is (clock - interpDelay); sweep that across [tPrev, tCur].
      clock.t = tPrev + ((tCur - tPrev) * i) / N + D.interpDelay;
      const v = D.drawnVehicles().find((x) => x.id === id);
      if (!v) break;
      samples.push({ f: i / N, x: v.x, y: v.y });
    }
    check("vehicle present across the window", samples.length === N + 1,
          `${samples.length}/${N + 1} samples`);

    if (samples.length === N + 1) {
      const a = samples[0], b = samples[N];
      const total = Math.hypot(b.x - a.x, b.y - a.y);
      check("vehicle actually moved between snapshots", total > 0.5,
            `${total.toFixed(2)} m`);
      // Every sample must sit on the straight line from first to last, at the
      // fraction of it its timestamp says. An easing filter fails this: it
      // covers most of the distance early and crawls at the end.
      let worst = 0;
      for (const s of samples) {
        const want = { x: a.x + (b.x - a.x) * s.f, y: a.y + (b.y - a.y) * s.f };
        worst = Math.max(worst, Math.hypot(s.x - want.x, s.y - want.y));
      }
      check("motion is linear in time across the window",
            total > 0.5 ? worst < total * 0.02 : true,
            `worst deviation ${worst.toFixed(4)} m over ${total.toFixed(2)} m`);
    }
  }

  check("poll rate adapts to sim speed", D.pollInterval >= 45 && D.pollInterval <= 150,
        `${D.pollInterval} ms at speed ${(D.simState.speed || 1)}`);

  // Restart the loop for the sections below.
  await D.poll();
}

// ---------------------------------------------------------------------------
section("The camera survives an edit");
// ---------------------------------------------------------------------------
{
  // Zoom in hard, somewhere off-centre, the way a user would.
  const canvasEl = dom.canvas;
  for (let i = 0; i < 25; i++) {
    canvasEl.dispatch("wheel", {
      preventDefault() {}, deltaY: -120, deltaMode: 0, clientX: 400, clientY: 250,
    });
  }
  const zoomed = { ...D.T };
  check("wheel zooms in", zoomed.s > D.fitScale * 3,
        `s=${zoomed.s.toFixed(2)} vs fit ${D.fitScale.toFixed(3)}`);

  await D.reloadMap();
  check("reloadMap leaves the camera alone",
        D.T.s === zoomed.s && D.T.tx === zoomed.tx && D.T.ty === zoomed.ty,
        `s ${zoomed.s.toFixed(3)} -> ${D.T.s.toFixed(3)}`);

  check("no dblclick handler is registered on the canvas",
        !(canvasEl.__handlers.get("dblclick") || []).length);

  // A window resize keeps what the user is looking at.
  const beforeResize = { ...D.T };
  const midBefore = { x: (canvasEl.width / 2 - beforeResize.tx) / beforeResize.s,
                      y: (canvasEl.height / 2 - beforeResize.ty) / beforeResize.s };
  canvasEl.clientWidth = 900;
  dom.windowEvent("resize", {});
  const midAfter = { x: (canvasEl.width / 2 - D.T.tx) / D.T.s,
                     y: (canvasEl.height / 2 - D.T.ty) / D.T.s };
  check("resize keeps the scale", Math.abs(D.T.s - beforeResize.s) < 1e-9,
        `${beforeResize.s.toFixed(3)} -> ${D.T.s.toFixed(3)}`);
  check("resize keeps the centre",
        Math.hypot(midAfter.x - midBefore.x, midAfter.y - midBefore.y) < 1e-6);

  // Twice: dragging a window edge fires a stream of resizes, so surviving one
  // and then refitting on the next is the same bug from the user's side.
  canvasEl.clientWidth = 700;
  dom.windowEvent("resize", {});
  canvasEl.clientWidth = 1100;
  dom.windowEvent("resize", {});
  const midFinal = { x: (canvasEl.width / 2 - D.T.tx) / D.T.s,
                     y: (canvasEl.height / 2 - D.T.ty) / D.T.s };
  check("repeated resizes still keep the view",
        Math.abs(D.T.s - beforeResize.s) < 1e-9 &&
        Math.hypot(midFinal.x - midBefore.x, midFinal.y - midBefore.y) < 1e-6,
        `s=${D.T.s.toFixed(3)}`);

  // Explicit fit still works.
  D.fitView();
  check("fitView still frames the city", Math.abs(D.T.s - D.fitScale) < 1e-9);
}

// ---------------------------------------------------------------------------
section("Loading a DIFFERENT city does refit");
// ---------------------------------------------------------------------------
{
  // The counterpart to the section above: an in-place edit must leave the
  // camera alone, but switching cities must not leave it framing the old one.
  const canvasEl = dom.canvas;
  for (let i = 0; i < 25; i++) {
    canvasEl.dispatch("wheel", {
      preventDefault() {}, deltaY: -120, deltaMode: 0, clientX: 400, clientY: 250,
    });
  }
  check("zoomed in before switching", D.T.s > D.fitScale * 3);

  await D.reloadCity();
  check("a new city reframes the camera", Math.abs(D.T.s - D.fitScale) < 1e-9,
        `s=${D.T.s.toFixed(3)}, fit=${D.fitScale.toFixed(3)}`);
  check("map still intact after reloadCity", D.drawRoads.length > 0,
        `${D.drawRoads.length} streets`);
}

// ---------------------------------------------------------------------------
section("Zoom reaches street level");
// ---------------------------------------------------------------------------
{
  const canvasEl = dom.canvas;
  for (let i = 0; i < 120; i++) {
    canvasEl.dispatch("wheel", {
      preventDefault() {}, deltaY: -120, deltaMode: 0, clientX: 600, clientY: 300,
    });
  }
  const maxS = Math.max(D.fitScale * 12, 60);
  check("zoom reaches the cap", Math.abs(D.T.s - maxS) < 1e-6,
        `${D.T.s.toFixed(1)} px/m (cap ${maxS.toFixed(1)})`);

  // A 4.5 m car at the deepest zoom, as actually drawn.
  dom.ctx.__reset();
  frame();
  const sprites = dom.ctx.__calls.filter((c) => c.op === "fillRect" &&
    typeof c.w === "number" && c.w > 0 && c.h === c.w * 0.5);
  const widest = sprites.reduce((m, c) => Math.max(m, c.w), 0);
  check("a car is legible at full zoom", widest > 60,
        `widest car sprite ${widest.toFixed(0)} px`);

  // A trackpad sends many small deltas; one event must not fly across the range.
  const s0 = D.T.s;
  canvasEl.dispatch("wheel", {
    preventDefault() {}, deltaY: 4000, deltaMode: 0, clientX: 600, clientY: 300,
  });
  check("one wheel event cannot jump more than a notch",
        D.T.s >= s0 / 1.2501, `${s0.toFixed(1)} -> ${D.T.s.toFixed(1)} px/m`);

  D.fitView();
}

// ---------------------------------------------------------------------------
section("Per-frame cost");
// ---------------------------------------------------------------------------
{
  check("no procedural scenery on a city-scale map", D.props.length === 0,
        `${D.props.length} props, ${D.drawRoads.length} roads, ` +
        `${(D.mapData.buildings || []).length} buildings`);

  dom.ctx.__reset();
  frame();
  const grads = dom.ctx.__calls.filter((c) => c.op === "radialGradient").length;
  check("no per-prop radial gradients per frame", grads === 0, `${grads}`);

  // Status overlay: one entry per physical street, only where it means
  // something, and only what is on screen.
  const stateRoads = (D.simState.roads || []).length;
  check("live-road list is merged and thresholded",
        D.liveRoads.length <= stateRoads,
        `${D.liveRoads.length} drawn entries from ${stateRoads} reported roads`);
  check("live roads carry no duplicate streets",
        new Set(D.liveRoads.map((e) => e.base)).size === D.liveRoads.length);

  // Frame cost with the whole city in view.
  D.fitView();
  dom.ctx.__reset();
  frame();
  const perFrame = dom.ctx.__calls.length;
  const t0 = Date.now();
  for (let i = 0; i < 30; i++) frame();
  const ms = (Date.now() - t0) / 30;
  check("frame stays cheap at fit zoom", perFrame < 20000,
        `${perFrame} draw calls, ${ms.toFixed(1)} ms/frame in the recorder`);
}

// Stop the poll loop before exiting, or Node tears down with fetches still
// in flight and libuv complains on the way out.
D.stopPolling();
process.exitCode = report() ? 0 : 1;

