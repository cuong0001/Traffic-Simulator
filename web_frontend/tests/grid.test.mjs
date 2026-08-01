// The other branch: a small hand-drawn-style city with 240 m blocks and no
// building footprints. render.test.mjs covers a real OpenStreetMap extract,
// where scenery is dropped and the unit scale bottoms out; here scenery is
// supposed to be scattered and the unit scale is supposed to be 1, so a change
// tuned for one shape of map cannot quietly break the other.
//
//   node web_frontend/tests/make_grid_map.mjs <tmp>/grid_map.geojson
//   build\traffic_simulator.exe <tmp>\grid_map.geojson 8097
//   node web_frontend/tests/grid.test.mjs http://localhost:8097

"use strict";

import { install, check, section, report } from "./harness.mjs";

const origin = process.argv[2] || "http://localhost:8097";
const dom = install({ origin });

const app = await import("../js/app.js");
const D = app.__debug;
await D.ready;

if (!D.mapData) {
  console.error(`No map. Is traffic_simulator serving ${origin}?`);
  process.exit(1);
}

const api = async (p) => (await fetch(origin + p)).json();

section("A small map keeps its scenery and its true scale");
{
  check("map is the small shape", D.drawRoads.length > 0 && D.drawRoads.length < 1500,
        `${D.drawRoads.length} streets, ${(D.mapData.buildings || []).length} buildings`);
  check("unit scale is 1 on 240 m blocks", Math.abs(D.US - 1) < 1e-3, `US=${D.US}`);
  check("vehicle scale follows it", Math.abs(D.VS - Math.max(D.US, 0.35)) < 1e-9,
        `VS=${D.VS}`);
  check("scenery IS scattered here", D.props.length > 0, `${D.props.length} props`);

  // The point of the occupancy grid: no tree may overlap a road ribbon. Check
  // it geometrically rather than trusting the grid that placed them.
  const distToSeg = (px, py, a, b) => {
    const vx = b.x - a.x, vy = b.y - a.y;
    const l2 = vx * vx + vy * vy;
    if (!l2) return Math.hypot(px - a.x, py - a.y);
    let t = ((px - a.x) * vx + (py - a.y) * vy) / l2;
    t = Math.max(0, Math.min(1, t));
    return Math.hypot(px - (a.x + t * vx), py - (a.y + t * vy));
  };
  let worst = Infinity, offender = null;
  for (const p of D.props) {
    const need = 30 * D.US * 0.5 + p.w * 0.5;   // half road + half canopy
    for (const r of D.drawRoads) {
      const a = D.nodeById[r.from], b = D.nodeById[r.to];
      if (!a || !b) continue;
      const slack = distToSeg(p.x, p.y, a, b) - need;
      if (slack < worst) { worst = slack; offender = p; }
    }
  }
  check("no tree overlaps a road", worst >= 0,
        `tightest clearance ${worst.toFixed(1)} m over ${D.props.length} props` +
        (offender && worst < 0 ? ` (${offender.name})` : ""));
}

section("...and everything else still works on it");
{
  const map = await api("/api/map");
  const twoWay = map.roads.filter((r) => !r.oneway && !r.id.endsWith("_r"));
  check("reverse twins synthesised", twoWay.every((r) => !!D.roadById[r.id + "_r"]),
        `${twoWay.length} streets`);

  const st = await api("/api/state");
  check("every vehicle resolves", st.vehicles.every((v) => !!D.roadById[v.road]),
        `${st.vehicles.length} vehicles`);
  check("every live vehicle is placed",
        D.drawnVehicles().length === (D.simState.vehicles || []).length,
        `${D.drawnVehicles().length}/${(D.simState.vehicles || []).length}`);

  let target = null;
  for (const v of D.simState.vehicles) {
    const d = await api("/api/vehicle/detail?id=" + encodeURIComponent(v.id));
    if (d.route && d.route.length >= 3) { target = v; break; }
  }
  if (target) {
    await D.selectVehicleRoute(target.id);
    dom.ctx.__reset();
    dom.frame(16);
    const routeStrokes = dom.ctx.__calls.filter(
      (c) => c.op === "stroke" && String(c.style).toLowerCase() === "#3f8fe0");
    const segs = routeStrokes.length
      ? routeStrokes[0].path.filter((p) => p[0] === "M").length : 0;
    check("route highlight draws every road", segs === D.routeRoadIds.length,
          `${segs}/${D.routeRoadIds.length}`);
    D.clearRouteHighlight();
  } else {
    check("found a route to draw", false, "no vehicle had one");
  }

  // Scenery must survive an in-place edit, since nothing rebuilds it.
  const before = D.props.length;
  const cam = { ...D.T };
  await D.reloadMap();
  check("an edit keeps the scenery", D.props.length === before, `${before} props`);
  check("an edit keeps the camera",
        D.T.s === cam.s && D.T.tx === cam.tx && D.T.ty === cam.ty);

  dom.ctx.__reset();
  dom.frame(16);
  check("frame draws something", dom.ctx.__calls.length > 0,
        `${dom.ctx.__calls.length} calls`);
}

// Stop the poll loop before exiting, or Node tears down with fetches still
// in flight and libuv complains on the way out.
D.stopPolling();
process.exitCode = report() ? 0 : 1;

