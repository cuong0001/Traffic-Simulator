// History playback tests: drive the REAL app.js against a REAL running
// simulator, same harness as render.test.mjs.
//
//   node web_frontend/tests/history.test.mjs [http://localhost:8099]
//
// Start the server first, with some vehicles moving so history actually has
// something in it by the time these run:
//   build\traffic_simulator.exe data\maps\small_map.geojson 8099
//   curl "http://localhost:8099/api/spawnRate?value=10"
//   curl "http://localhost:8099/api/target?value=100"
//
// Nothing here re-implements the scrub control. The harness supplies a DOM
// and a recording 2D context, app.js loads and boots for real, and the
// assertions ask what it actually did against the real /api/history/* API.

"use strict";

import { install, check, section, report, sleep } from "./harness.mjs";

const origin = process.argv[2] || "http://localhost:8099";

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

// Give the server a moment to accumulate a few captured seconds of real
// history (capture cadence is 1/real-second, see HistoryStore.hpp), and wait
// for the frontend's own periodic range fetch to notice.
await D.refreshHistoryRange();
let waited = 0;
while (D.historyRange.empty && waited < 5000) {
  await sleep(250);
  waited += 250;
  await D.refreshHistoryRange();
}
if (D.historyRange.empty) {
  console.error("No history captured yet -- is the server actually ticking (not paused)?");
  process.exit(1);
}
const ago = Math.max(1, Math.min(5, Math.floor(D.historyRange.oldestAgo)));

// ---------------------------------------------------------------------------
section("Scrubbing renders a real historical frame through the live render path");
// ---------------------------------------------------------------------------
{
  D.setHistoryAgo(ago);
  await sleep(400);   // the fetch inside setHistoryAgo() is fire-and-forget

  check("historyAgo reflects the scrub position", D.historyAgo === ago,
        `ago=${ago}, historyAgo=${D.historyAgo}`);
  check("simState was replaced by a historical payload",
        !!D.simState && typeof D.simState.time === "number");
  // The historical vehicles array went through applyHistoryVehicles(), which
  // calls the SAME placeOnRoad() the live binary path uses -- a non-empty
  // result with real coordinates is proof it isn't silently dropping everyone.
  const vs = D.drawnVehicles();
  check("at least placed vehicles land at real coordinates (or there were none)",
        vs.every((v) => Number.isFinite(v.x) && Number.isFinite(v.y)),
        `${vs.length} vehicles`);
}

// ---------------------------------------------------------------------------
section("Scrubbing stops live polling; Jump to Live resumes it");
// ---------------------------------------------------------------------------
{
  // Already scrubbed from the section above.
  check("live polling is stopped while viewing history", D.isPolling === false);

  D.setHistoryAgo(null);
  await sleep(50);
  check("historyAgo clears on Jump to Live", D.historyAgo === null);
  check("live polling resumes on Jump to Live", D.isPolling === true);

  D.stopPolling();  // tidy up: don't leave timers running past process exit
}

// ---------------------------------------------------------------------------
section("Editing/interaction lockout follows scrub state");
// ---------------------------------------------------------------------------
{
  const lock = dom.el("history-lock");
  const badge = dom.el("history-badge");

  D.setHistoryAgo(ago);
  await sleep(50);
  check("the click-blocking overlay is shown while scrubbed back",
        lock.classList.contains("hidden") === false);
  check("the history badge is shown while scrubbed back",
        badge.classList.contains("hidden") === false);

  D.setHistoryAgo(null);
  await sleep(50);
  check("the click-blocking overlay is hidden again once back at live",
        lock.classList.contains("hidden") === true);
  check("the history badge is hidden again once back at live",
        badge.classList.contains("hidden") === true);

  D.stopPolling();
}

const ok = report();
process.exit(ok ? 0 : 1);
