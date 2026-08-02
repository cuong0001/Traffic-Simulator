// Chrome for the simulator: menu bar, slide-in sidebar, stats drawer, event
// log, selection card and toasts. Ported from the reference UI and adapted to
// this repo's API surface.
//
// The renderer (app.js) owns the canvas and the simulation state; this module
// only touches DOM and calls back into the hooks it is given, so the two can be
// reasoned about separately.

"use strict";

import { Sim } from "./api.js";

const $ = (id) => document.getElementById(id);

// Opening one of these arms a map tool: the next canvas click is routed to
// handleMapClick() instead of the inspector. Esc (or closing the sidebar)
// disarms it.
const TOOL_PANELS = new Set([
  "add-building", "add-road", "add-light",
  "del-road", "del-building", "del-light", "del-vehicle", "del-deco",
  "mod-vehicle", "mod-road", "mod-junction",
]);

const PANEL_TITLE = {
  "map-details": "Map Details",
  "adjust": "Adjust Simulation",
  "add-building": "Add Building",
  "add-road": "Add Road",
  "add-vehicle": "Add Vehicle",
  "add-light": "Add Traffic Lights",
  "del-road": "Delete Roads",
  "del-building": "Delete Buildings",
  "del-light": "Delete Traffic Lights",
  "del-vehicle": "Delete Vehicles",
  "del-deco": "Delete Decoration",
  "mod-vehicle": "Modify Vehicles",
  "mod-road": "Modify Roads",
  "mod-junction": "Modify Intersections",
  "incident-fire": "Fire",
  "incident-medical": "Medical Emergency",
  "incident-robbery": "Robbery",
  "incident-smash": "Smash Road",
  "deco-tree": "Tree",
  "deco-grass": "Grass",
  "files-open": "Open Map",
  "files-save": "Save Map",
};

let hooks = {};                 // supplied by app.js
let congHistory = [];           // rolling average congestion, for the chart
let toastTimer = null;
let selectedVehicleType = null;

// --- map-tool state -------------------------------------------------------
let tool = null;                // active panel name, or null
let buildingKind = null;        // chosen type in "Add Building"
let roadFirst = null;           // first endpoint of "Add Road"
const picked = {};              // tool -> selected element id

// ---------------------------------------------------------------------------
// Small shared helpers
// ---------------------------------------------------------------------------
export function toast(msg) {
  const el = $("toast");
  if (!el) return;
  el.textContent = msg;
  el.classList.remove("hidden");
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => el.classList.add("hidden"), 2200);
}

export function log(msg) {
  const ul = $("log");
  if (!ul) return;
  const li = document.createElement("li");
  const t = hooks.getState?.()?.time || 0;
  const mm = String(Math.floor(t / 60)).padStart(2, "0");
  const ss = String(Math.floor(t % 60)).padStart(2, "0");
  li.textContent = `[${mm}:${ss}] ${msg}`;
  ul.prepend(li);
  while (ul.children.length > 60) ul.lastChild.remove();
}

function fmtClock(t) {
  const m = Math.floor((t || 0) / 60), s = Math.floor((t || 0) % 60);
  return `${m}:${String(s).padStart(2, "0")}`;
}

// ---------------------------------------------------------------------------
// Sidebar
// ---------------------------------------------------------------------------
function openPanel(name) {
  const bar = $("right-sidebar");
  for (const p of document.querySelectorAll(".sb-panel")) p.classList.remove("active");
  const panel = $("panel-" + name);
  if (panel) panel.classList.add("active");
  $("sidebar-title").textContent = PANEL_TITLE[name] || "Tools";
  bar.classList.add("active");

  hooks.setPlacing?.(null);
  roadFirst = null;
  tool = TOOL_PANELS.has(name) ? name : null;
  if (tool) toast("Click the map — Esc to stop");

  if (name === "map-details") renderMapDetails();
  if (name === "files-open") refreshMapList();
  if (name === "add-road") $("addroad-status").textContent = "Pick the first point…";
}

function closeSidebar() {
  $("right-sidebar").classList.remove("active");
  disarm();
}

function disarm() {
  tool = null;
  roadFirst = null;
  hooks.setPlacing?.(null);
}

// True only while the Add Road tool is armed, so the canvas can draw the
// rubber-band endpoint marker for that tool and no other. (There used to be an
// isArmed() beside this, for "any tool", whose only caller was the canvas
// deciding whether a double-click should refit the view. Double-click no longer
// does anything, so it went with it.)
export function isAddingRoad() { return tool === "add-road"; }

// The OSM picker covers the whole stage, and the event log floats above it.
// Tuck it away while the picker is open and put it back exactly as it was.
let logWasVisible = false;
export function suspendEventLog() {
  const box = $("eventlog-box");
  if (!box) return;
  logWasVisible = !box.classList.contains("hidden");
  box.classList.add("hidden");
}
export function restoreEventLog() {
  const box = $("eventlog-box");
  if (!box || !logWasVisible) return;
  box.classList.remove("hidden");
}

// The first endpoint of an in-progress "Add Road", or null. The canvas asks so
// it can draw the rubber-band line to the cursor; without it the user picks the
// second point blind and only finds out what they connected afterwards.
export function pendingRoadStart() { return tool === "add-road" ? roadFirst : null; }

// Routed from app.js on every canvas click. Returns true when the click was
// consumed by a tool, so the inspector stays out of the way.
export function handleMapClick(x, y, hit, propIndex) {
  if (!tool) return false;

  switch (tool) {
    case "add-building": {
      if (!buildingKind) { toast("Pick a building type first"); return true; }
      Sim.addBuilding(x, y, buildingKind, "", 24).then(async (r) => {
        if (!r || r.ok === false) return toast("Could not add building");
        log(`Added ${buildingKind} ${r.id}`);
        await hooks.reloadMap?.();
      });
      return true;
    }

    case "add-road": {
      // Each end is a junction, a point on a road (split it), or free ground.
      const end = hit && hit.kind === "junction" ? { kind: "junction", id: hit.id }
                : hit && hit.kind === "road"     ? { kind: "road", x, y }
                                                 : { kind: "free", x, y };
      if (!roadFirst) {
        roadFirst = end;
        $("addroad-status").textContent = `First: ${end.kind}. Pick the second point…`;
        return true;
      }
      const opts = {
        speed: Number($("addroad-speed").value) || 40,
        lanes: Number($("addroad-lanes").value) || 1,
        oneway: !$("addroad-twoway").checked,
      };
      const first = roadFirst;
      roadFirst = null;
      $("addroad-status").textContent = "Creating…";
      Sim.addRoad(first, end, opts).then(async (r) => {
        if (!r || r.ok === false) {
          $("addroad-status").textContent = "Failed. Pick the first point…";
          return toast(r?.error || "Could not create road");
        }
        $("addroad-status").textContent = `Created ${r.id}. Pick the first point…`;
        log(`New road ${r.id} (${r.from} → ${r.to})` + (r.forcedTwoWay ? " — forced two-way" : ""));
        await hooks.reloadMap?.();
      });
      return true;
    }

    case "add-light":
    case "del-light": {
      if (!hit || hit.kind !== "junction") { toast("Click a junction"); return true; }
      const on = tool === "add-light";
      const green = Number($("addlight-green").value) || 25;
      Sim.setLight(hit.id, on, green).then(async (r) => {
        if (!r || r.ok === false) return toast("Could not change the light");
        log(`${on ? "Installed" : "Removed"} signals at ${hit.id}`);
        await hooks.reloadMap?.();
      });
      return true;
    }

    case "del-road": {
      if (!hit || hit.kind !== "road") { toast("Click a road"); return true; }
      picked["del-road"] = hit.id;
      $("delroad-target").textContent = hit.id;
      return true;
    }

    case "del-building": {
      if (!hit || hit.kind !== "building") { toast("Click a building"); return true; }
      Sim.removeBuilding(hit.id).then(async (r) => {
        if (!r || r.ok === false) return toast("Could not demolish");
        log(`Demolished ${hit.id}`);
        await hooks.reloadMap?.();
      });
      return true;
    }

    case "del-vehicle":
    case "mod-vehicle": {
      if (!hit || hit.kind !== "vehicle") { toast("Click a vehicle"); return true; }
      picked[tool] = hit.id;
      $(tool === "del-vehicle" ? "delveh-target" : "modveh-target").textContent = hit.id;
      return true;
    }

    case "del-deco": {
      if (hooks.removePropAt?.(propIndex)) log("Removed decoration");
      else toast("No decoration there");
      return true;
    }

    case "mod-road": {
      if (!hit || hit.kind !== "road") { toast("Click a road"); return true; }
      picked["mod-road"] = hit.id;
      $("modroad-target").textContent = hit.id;
      $("modroad-speed").value = hit.road.speed_kmh || 40;
      return true;
    }

    case "mod-junction": {
      if (!hit || hit.kind !== "junction") { toast("Click a junction"); return true; }
      picked["mod-junction"] = hit.id;
      $("junction-target").textContent = hit.id;
      $("junction-light").checked = !!hit.node.signal;
      return true;
    }
  }
  return false;
}

async function refreshMapList() {
  const box = $("maps-list");
  if (!box) return;
  const r = await Sim.maps();
  const names = (r && r.maps) || [];
  if (!names.length) { box.innerHTML = `<p class="sb-hint">No maps found.</p>`; return; }
  box.innerHTML = names
    .map((n) => `<div class="selectable-item" data-map="${n}"><span>${n}</span></div>`)
    .join("");
  for (const el of box.querySelectorAll("[data-map]")) {
    el.addEventListener("click", async () => {
      const name = el.dataset.map;
      const res = await Sim.loadMap(name);
      if (!res || res.ok === false) return toast("Could not load " + name);
      log(`Loaded map ${name}`);
      // reloadCity, not reloadMap: a different city needs its scenery rebuilt
      // and the camera reframed, where an in-place edit needs neither.
      await hooks.reloadCity?.();
      hooks.poll?.();
    });
  }
}

function renderMapDetails() {
  const m = hooks.getMap?.();
  const body = $("map-details-body");
  if (!m || !body) return;
  const signals = m.intersections.filter((n) => n.signal).length;
  const cells = [
    ["Intersections", m.intersections.length.toLocaleString()],
    ["Roads", (hooks.getDrawRoads?.() || m.roads).length.toLocaleString()],
    ["Buildings", (m.buildings || []).length.toLocaleString()],
    ["Signals", signals.toLocaleString()],
    ["Zones", (m.zones || []).length.toLocaleString()],
    ["Scenery", String(hooks.getPropCount?.() ?? 0)],
  ];
  body.innerHTML = cells
    .map(([k, v]) => `<div><span class="k">${k}</span><span class="v">${v}</span></div>`)
    .join("");
}

// ---------------------------------------------------------------------------
// Stats drawer
// ---------------------------------------------------------------------------
function renderStats(state) {
  if ($("stats").classList.contains("hidden") || !state) return;
  const st = state.stats || {};
  const vs = state.vehicles || [];
  const set = (id, v) => { const e = $(id); if (e) e.textContent = v; };

  // Aggregates come from StatisticsManager under "stats"; the live counts the
  // simulator owns itself sit at the top level of the state payload.
  set("st-trips", st.completedTrips ?? 0);
  set("st-spawned", state.spawned ?? 0);
  set("st-active", state.active ?? vs.length);
  set("st-reroutes", st.routeComputations ?? 0);
  set("st-events", (state.events || []).length);

  const avg = vs.length ? vs.reduce((a, v) => a + (v.speed || 0), 0) / vs.length : 0;
  set("st-speed", `${(avg * 3.6).toFixed(0)} km/h`);

  // Traffic-solver counters (MOBIL lane changes, M7 gridlock relief).
  const sv = state.solver || {};
  set("st-lanes", sv.lane_changes ?? 0);
  set("st-forced", sv.forced ?? 0);
  set("st-rescues", sv.rescues ?? 0);

  // Per-algorithm table: only populated once the backend reports algorithm
  // metrics (StatisticsManager.algorithmStats). Until then say so plainly.
  const tbody = document.querySelector("#algo-table tbody");
  if (tbody) {
    const algos = st.algorithms || {};
    const rows = Object.entries(algos);
    tbody.innerHTML = rows.length
      ? rows.map(([name, a]) => `<tr><td>${name}</td><td>${a.computations ?? 0}</td>` +
          `<td>${(a.avgMicros ?? 0).toFixed(1)}</td><td>${(a.avgNodesExplored ?? 0).toFixed(0)}</td>` +
          `<td>${a.failures ?? 0}</td></tr>`).join("")
      : `<tr><td colspan="5" style="text-align:left;color:#8fa3b8">no algorithm metrics yet</td></tr>`;
  }
  drawChart();
}

// The traffic events themselves are logged by TrafficController, which sees
// every start and expiry. Polling can miss a short event entirely, so the
// EVENTS panel replays the server's log rather than guessing from the diff of
// two /api/state snapshots.
let lastServerLogLine = null;

function pushServerLog(state) {
  const lines = state.log || [];
  if (!lines.length) return;
  const seen = lastServerLogLine === null ? -1 : lines.lastIndexOf(lastServerLogLine);
  // seen < 0 means the log rotated past everything we had shown: replay it all.
  for (const line of lines.slice(seen + 1)) log(line);
  lastServerLogLine = lines[lines.length - 1];
}

function pushCongestion(state) {
  // The backend samples every road every two seconds and keeps the history, so
  // prefer that over averaging what arrives in `state.roads` -- that list only
  // carries roads that are congested or closed, which would make the average
  // read far higher than the network really is.
  const history = state.stats && state.stats.history;
  let avg;
  if (history && history.length) {
    congHistory = history.slice(-240).map((s) => s.congestion || 0);
    avg = congHistory[congHistory.length - 1];
  } else {
    const roads = state.roads || [];
    avg = roads.length
      ? roads.reduce((a, r) => a + (r.congestion || 0), 0) / roads.length
      : 0;
    congHistory.push(avg);
    if (congHistory.length > 240) congHistory.shift();
  }
  const el = $("hud-congestion");
  if (el) el.textContent = `${Math.round(avg * 100)}%`;
}

function drawChart() {
  const c = $("chart");
  if (!c) return;
  const g = c.getContext("2d");
  g.clearRect(0, 0, c.width, c.height);
  if (congHistory.length < 2) return;
  g.strokeStyle = "#e8a33d";
  g.lineWidth = 2;
  g.beginPath();
  const n = congHistory.length;
  for (let i = 0; i < n; i++) {
    const x = (i / (n - 1)) * c.width;
    const y = c.height - Math.min(1, congHistory[i]) * (c.height - 4) - 2;
    i ? g.lineTo(x, y) : g.moveTo(x, y);
  }
  g.stroke();
}

// ---------------------------------------------------------------------------
// Per-poll refresh
// ---------------------------------------------------------------------------
// Mirror the simulator's own settings into the controls. Only touched when the
// user is not interacting with the control in question -- overwriting a number
// someone is halfway through typing is worse than showing a stale one.
let settingsSynced = false;
function syncSettings(state) {
    if (!state) return;
    const target = $("target-input");
    if (target && document.activeElement !== target &&
        state.targetVehicles !== undefined &&
        Number(target.value) !== state.targetVehicles) {
        // After the first sync, only follow the server when the user is not the
        // one who changed it -- otherwise a slow round trip snaps the box back.
        if (!settingsSynced) target.value = state.targetVehicles;
    }
    const rate = $("spawnrate-slider");
    if (rate && !settingsSynced && state.spawnRate !== undefined) {
        rate.value = state.spawnRate;
        $("spawnrate-val").textContent = `${Number(state.spawnRate).toFixed(1)}/s`;
    }
    // Derived from the map on load, so show what the server actually chose
    // rather than whatever the markup happened to say.
    const appr = $("approach-slider");
    if (appr && !settingsSynced && state.junctionApproach !== undefined) {
        appr.value = Math.round(state.junctionApproach);
        $("approach-val").textContent = `${Math.round(state.junctionApproach)} m`;
    }
    const algo = $("algo-select");
    if (algo && !settingsSynced && state.algorithm) algo.value = state.algorithm;
    const rnd = $("chk-random");
    if (rnd && !settingsSynced && state.randomEvents !== undefined) {
        rnd.checked = !!state.randomEvents;
    }
    const spawn = $("chk-autospawn");
    if (spawn && !settingsSynced && state.autoSpawn !== undefined) {
        spawn.checked = !!state.autoSpawn;
    }
    settingsSynced = true;
}

export function update(state) {
  syncSettings(state);
  const set = (id, v) => { const e = $(id); if (e) e.textContent = v; };
  if (!state) {
    set("hud-time", "--:--");
    set("hud-vehicles", "0");
    set("hud-congestion", "0%");
    return;
  }
  set("hud-time", fmtClock(state.time));
  set("hud-vehicles", String((state.vehicles || []).length));
  const btn = $("btn-pause");
  if (btn) btn.innerHTML = state.paused ? "&#9654; Resume" : "&#10074;&#10074; Pause";
  pushCongestion(state);
  pushServerLog(state);
  renderStats(state);
}

// ---------------------------------------------------------------------------
// Selection card
// ---------------------------------------------------------------------------
export function showSelection(title, rows) {
  $("sel-title").textContent = title;
  $("sel-body").innerHTML = rows.map(([k, v]) => `<p>${k}: <b>${v}</b></p>`).join("");
  $("selection").classList.remove("hidden");
}
export function hideSelection() { $("selection").classList.add("hidden"); }

// ---------------------------------------------------------------------------
// Wiring
// ---------------------------------------------------------------------------
export function init(h) {
  hooks = h || {};

  // --- menu bar: any button carrying data-panel opens that sidebar panel ---
  for (const btn of document.querySelectorAll("#menubar [data-panel]")) {
    btn.addEventListener("click", () => openPanel(btn.dataset.panel));
  }
  $("sidebar-close").addEventListener("click", closeSidebar);

  // --- View menu ---
  const toggle = (el, btn) => {
    el.classList.toggle("hidden");
    btn.classList.toggle("on", !el.classList.contains("hidden"));
  };
  $("toggle-stats").addEventListener("click", (e) => {
    toggle($("stats"), e.currentTarget);
    renderStats(hooks.getState?.());
  });
  $("toggle-eventlog").addEventListener("click", (e) => toggle($("eventlog-box"), e.currentTarget));
  $("toggle-fps").addEventListener("click", (e) => toggle($("fpsmeter"), e.currentTarget));
  $("menu-fit").addEventListener("click", () => hooks.fitView?.());
  $("sel-close").addEventListener("click", hideSelection);

  // --- Files ---
  $("menu-save").addEventListener("click", async () => {
    const r = await Sim.saveMap("autosave");
    toast(r && r.ok !== false ? "Saved as autosave" : "Save failed");
    if (r && r.ok !== false) log("Map saved as autosave");
  });
  $("save-go").addEventListener("click", async () => {
    const name = ($("save-name").value || "").trim();
    if (!name) return toast("Enter a name first");
    const r = await Sim.saveMap(name);
    if (!r || r.ok === false) return toast("Save failed (name must be a plain filename)");
    log(`Map saved as ${name}`);
    toast(`Saved ${name}`);
  });

  // --- pause / resume ---
  $("btn-pause").addEventListener("click", async () => {
    const s = hooks.getState?.();
    if (!s) return toast("Simulation offline");
    await (s.paused ? Sim.resume() : Sim.pause());
    log(s.paused ? "Simulation resumed" : "Simulation paused");
    hooks.poll?.();
  });

  // --- Adjust Simulation ---
  const speed = $("speed-slider");
  speed.addEventListener("input", () => {
    // reference mapping: slider -1..1 -> 0.2x .. 5x, centre is exactly 1.0x
    const factor = Math.pow(5, Number(speed.value));
    $("speed-val").textContent = `${factor.toFixed(1)}x`;
    Sim.setSpeed(factor);
  });
  $("chk-random").addEventListener("change", (e) => {
    Sim.setRandomEvents(e.target.checked);
    log(`Random events ${e.target.checked ? "on" : "off"}`);
  });
  $("chk-autospawn").addEventListener("change", (e) => {
    Sim.setAuto(e.target.checked);
    log(`Auto-spawn ${e.target.checked ? "on" : "off"}`);
  });

  const rate = $("spawnrate-slider");
  rate.addEventListener("input", () => {
    const v = Number(rate.value);
    $("spawnrate-val").textContent = `${v.toFixed(1)}/s`;
    Sim.setSpawnRate(v);
  });

  const approach = $("approach-slider");
  if (approach) {
    approach.addEventListener("input", () => {
      const v = Number(approach.value);
      $("approach-val").textContent = `${v} m`;
      Sim.setJunctionApproach(v);
    });
  }

  $("target-apply").addEventListener("click", async () => {
    const v = Math.max(0, Number($("target-input").value) || 0);
    await Sim.setTarget(v);
    log(v === 0 ? "Vehicle cap: unlimited" : `Vehicle cap: ${v}`);
    toast(v === 0 ? "Unlimited vehicles" : `Cap set to ${v}`);
  });

  $("algo-select").addEventListener("change", async (e) => {
    const r = await Sim.setAlgorithm(e.target.value);
    if (!r || r.ok === false) return toast("Unknown algorithm");
    log(`Routing algorithm: ${e.target.value}`);
  });

  // --- Add Building: choose a type, then click the map ---
  for (const item of document.querySelectorAll("#panel-add-building .selectable-item")) {
    item.addEventListener("click", () => {
      for (const o of document.querySelectorAll("#panel-add-building .selectable-item")) o.classList.remove("on");
      item.classList.add("on");
      buildingKind = item.dataset.bkind;
      toast(`Click the map to place a ${item.dataset.bname}`);
    });
  }

  // --- Add Traffic Lights: live green-duration readout ---
  const green = $("addlight-green");
  green.addEventListener("input", () => { $("addlight-green-val").textContent = `${green.value}s`; });
  const jgreen = $("junction-green");
  jgreen.addEventListener("input", () => { $("junction-green-val").textContent = `${jgreen.value}s`; });

  // --- Delete / Modify actions on the currently picked element ---
  $("delroad-go").addEventListener("click", async () => {
    const id = picked["del-road"];
    if (!id) return toast("Click a road on the map first");
    const r = await Sim.removeRoad(id);
    if (!r || r.ok === false) return toast("Could not delete the road");
    log(`Deleted road ${id}`);
    picked["del-road"] = null;
    $("delroad-target").textContent = "none selected";
    await hooks.reloadMap?.();
  });

  $("delveh-go").addEventListener("click", async () => {
    const id = picked["del-vehicle"];
    if (!id) return toast("Click a vehicle on the map first");
    const r = await Sim.removeVehicle(id);
    if (!r || r.ok === false) return toast("Could not delete the vehicle");
    log(`Deleted vehicle ${id}`);
    picked["del-vehicle"] = null;
    $("delveh-target").textContent = "none";
    hooks.poll?.();
  });

  $("delveh-all").addEventListener("click", async () => {
    const r = await Sim.clearVehicles();
    log(`Removed ${r?.removed ?? 0} vehicles`);
    toast(`Removed ${r?.removed ?? 0} vehicles`);
    hooks.poll?.();
  });

  $("modroad-go").addEventListener("click", async () => {
    const id = picked["mod-road"];
    if (!id) return toast("Click a road on the map first");
    const r = await Sim.updateRoad(id, Number($("modroad-speed").value) || 0,
                                   $("modroad-closed").checked);
    if (!r || r.ok === false) return toast("Could not update the road");
    log(`Updated ${id}` + ($("modroad-closed").checked ? " (closed)" : ""));
    await hooks.reloadMap?.();
  });

  $("modveh-algo-go").addEventListener("click", async () => {
    const id = picked["mod-vehicle"];
    if (!id) return toast("Click a vehicle on the map first");
    const r = await Sim.updateVehicle(id, { algorithm: $("modveh-algo").value });
    if (!r || r.ok === false) return toast("Could not change the algorithm");
    log(`${id} now routes with ${$("modveh-algo").value}`);
  });

  $("modveh-dest").addEventListener("click", async () => {
    const id = picked["mod-vehicle"];
    if (!id) return toast("Click a vehicle on the map first");
    const to = hooks.randomNode();
    const r = await Sim.updateVehicle(id, { to });
    if (!r || r.ok === false) return toast("No route to that destination");
    log(`${id} rerouted to ${to}`);
    hooks.poll?.();
  });

  $("junction-apply").addEventListener("click", async () => {
    const id = picked["mod-junction"];
    if (!id) return toast("Click a junction on the map first");
    const r = await Sim.setLight(id, $("junction-light").checked,
                                 Number($("junction-green").value) || 25);
    if (!r || r.ok === false) return toast("Could not update the junction");
    log(`${id}: signals ${$("junction-light").checked ? "on" : "off"}`);
    await hooks.reloadMap?.();
  });

  $("junction-delete").addEventListener("click", async () => {
    const id = picked["mod-junction"];
    if (!id) return toast("Click a junction on the map first");
    const r = await Sim.removeIntersection(id);
    if (!r || r.ok === false) return toast("Could not delete the junction");
    log(`Deleted junction ${id} and its roads`);
    picked["mod-junction"] = null;
    $("junction-target").textContent = "none";
    await hooks.reloadMap?.();
  });

  // --- Add Vehicle: list -> form -> add ---
  for (const item of document.querySelectorAll("#vehicle-list .selectable-item")) {
    item.addEventListener("click", () => {
      for (const o of document.querySelectorAll("#vehicle-list .selectable-item")) o.classList.remove("on");
      item.classList.add("on");
      selectedVehicleType = item.dataset.vtype;
      $("vehicle-list").classList.add("hidden");
      $("vehicle-form").classList.remove("hidden");
      $("vehicle-form-img").src = item.querySelector("img").src;
      $("vehicle-form-name").textContent = item.querySelector("span").textContent;
    });
  }
  $("vehicle-back").addEventListener("click", () => {
    $("vehicle-form").classList.add("hidden");
    $("vehicle-list").classList.remove("hidden");
  });
  $("vehicle-add").addEventListener("click", async () => {
    const n = Math.max(1, Math.min(100, Number($("vehicle-count").value) || 1));
    let ok = 0;
    for (let i = 0; i < n; i++) {
      const r = await Sim.spawn(hooks.randomNode(), hooks.randomNode(), selectedVehicleType);
      if (r && r.id) ok++;
    }
    log(`Added ${ok}/${n} ${selectedVehicleType}`);
    toast(ok ? `Spawned ${ok} ${selectedVehicleType}` : "Spawn failed (no route)");
    hooks.poll?.();
  });

  // --- Incidents. All four map onto the endpoints we do have. ---
  const pickRoad = (labelId) => {
    const id = hooks.randomRoad();
    const el = $(labelId);
    if (el) el.textContent = id;
    return id;
  };

  $("fire-go").addEventListener("click", async () => {
    const road = pickRoad("fire-target");
    await Sim.event("accident", road, 60);
    for (let i = 0; i < 2; i++) await Sim.spawn(hooks.randomNode(), hooks.randomNode(), "emergency");
    log(`Fire on ${road}: 2 units dispatched`);
    toast("Fire started");
    hooks.poll?.();
  });

  $("medical-go").addEventListener("click", async () => {
    const r = await Sim.spawn(hooks.randomNode(), hooks.randomNode(), "emergency");
    log(r && r.id ? `Ambulance ${r.id} dispatched` : "Ambulance failed (no route)");
    toast(r && r.id ? "Ambulance dispatched" : "No route available");
    hooks.poll?.();
  });

  $("robbery-go").addEventListener("click", async () => {
    const road = pickRoad("robbery-target");
    await Sim.event("congestion", road, 45);
    for (let i = 0; i < 2; i++) await Sim.spawn(hooks.randomNode(), hooks.randomNode(), "emergency");
    log(`Robbery on ${road}: 2 units responding`);
    toast("Robbery triggered");
    hooks.poll?.();
  });

  $("smash-go").addEventListener("click", async () => {
    const road = pickRoad("smash-target");
    const dur = Math.max(10, Number($("smash-duration").value) || 120);
    await Sim.event("closure", road, dur);
    log(`${road} closed for ${dur}s`);
    toast("Road smashed");
    hooks.poll?.();
  });

  // --- Decorations: client-side scenery, placed by the renderer ---
  for (const item of document.querySelectorAll("#panel-deco-tree .selectable-item, #panel-deco-grass .selectable-item")) {
    item.addEventListener("click", () => {
      for (const o of document.querySelectorAll(".selectable-item[data-prop]")) o.classList.remove("on");
      item.classList.add("on");
      hooks.setPlacing?.(item.dataset.prop);
      toast(`Click the map to place ${item.dataset.prop} — Esc to stop`);
    });
  }
  document.addEventListener("keydown", (e) => {
    if (e.key !== "Escape") return;
    disarm();
    hideSelection();
    for (const o of document.querySelectorAll(".selectable-item")) o.classList.remove("on");
    toast("Tool cancelled");
  });

  // pre-fill incident targets once the map is ready
  if (hooks.randomRoad) {
    for (const id of ["fire-target", "robbery-target", "smash-target"]) {
      const el = $(id);
      if (el) el.textContent = hooks.randomRoad();
    }
  }

  log("UI ready");
}
