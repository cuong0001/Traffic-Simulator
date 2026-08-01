// Writes a small hand-drawn-style grid city, the shape the renderer's other
// branch is tuned for: ~240 m blocks, no building footprints, well under the
// road count at which procedural scenery is dropped. The two maps we ship are
// both OpenStreetMap extracts, so without this the "scatter scenery, US = 1"
// path never gets exercised.
//
//   node web_frontend/tests/make_grid_map.mjs data/maps/grid_map.geojson

"use strict";
import { writeFileSync } from "node:fs";

const out = process.argv[2] || "data/maps/grid_map.geojson";
const N = 6;              // 6x6 junctions
const BLOCK = 240;        // metres between them
const LAT0 = 10.77, LON0 = 106.68;          // somewhere in Ho Chi Minh City
const dLat = BLOCK / 111320;
const dLon = BLOCK / (111320 * Math.cos((LAT0 * Math.PI) / 180));

const features = [];
const pt = (r, c) => [ +(LON0 + c * dLon).toFixed(7), +(LAT0 + r * dLat).toFixed(7) ];

for (let r = 0; r < N; r++) {
  for (let c = 0; c < N; c++) {
    if (c + 1 < N) features.push(street(pt(r, c), pt(r, c + 1)));
    if (r + 1 < N) features.push(street(pt(r, c), pt(r + 1, c)));
  }
}
// A few signalised crossings, so the signal-head path is exercised too.
for (let r = 1; r < N; r += 2) {
  for (let c = 1; c < N; c += 2) {
    features.push({
      type: "Feature",
      properties: { highway: "traffic_signals" },
      geometry: { type: "Point", coordinates: pt(r, c) },
    });
  }
}

function street(a, b) {
  return {
    type: "Feature",
    properties: { highway: "residential", lanes: "2", maxspeed: "50" },
    geometry: { type: "LineString", coordinates: [a, b] },
  };
}

writeFileSync(out, JSON.stringify({ type: "FeatureCollection", features }));
console.log(`${out}: ${features.length} features (${N}x${N} grid, ${BLOCK} m blocks)`);
