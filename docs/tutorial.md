# Tutorial: using the simulator

A walkthrough of the running app — every menu, every tool, in the order
you'll actually reach for them. If something here doesn't match what you
see on screen, the UI has moved on since this was written; treat the app
as the source of truth and file an issue rather than trust this page.

## 1. First run

Build and start the server per the [README](../README.md#building-and-running)
— the short version:

```
cmake -S . -B build -G Ninja
cmake --build build
build/traffic_simulator data/maps/small_map.geojson 8080
```

Then open **http://localhost:8080**. Within a second or two you should see
vehicles moving, signals cycling between red/amber/green, and the top menu
bar. If nothing appears, check the terminal the server is running in — a
bad map path fails there with a clear message before the HTTP server ever
starts.

## 2. The screen

- **Menu bar** (top): `Files`, `Map`, `Incident`, `Decorations`, `View`,
  each a dropdown — covered section by section below — plus three HUD
  chips on the right (**Time**, **Veh**, **Jam**) and a **Pause** button.
  Time is the simulator's own clock, not a wall clock (more on this in
  §9); Veh is the live vehicle count; Jam is average road congestion.
- **The canvas**: drag to pan, scroll to zoom, `F` to fit the whole city
  back in view. Roads tint from grey toward red as they get more
  congested; a red dashed outline marks a closed road.
- **Right sidebar**: every tool below opens here. It slides in when you
  pick a menu item and closes with the `×` in its header.

## 3. Adjust Simulation

`Map → Adjust Simulation`. Every control here takes effect live, while
traffic keeps moving:

- **Speed** — 0.1x to 10x. Below 1x everything runs in slow motion; above
  it, faster. The simulator sub-steps its physics internally so cranking
  this up doesn't let fast-moving vehicles tunnel through junctions.
- **Spawn/s** — how many new vehicles enter the city per second, while
  auto-spawn (below) is on.
- **Max veh** — a cap on how many vehicles exist at once; 0 means
  unlimited. Type a number and hit **Set**.
- **Give way at** — how many metres before an unsignalized junction a
  driver starts deciding whether to yield. Smaller reads as more
  realistic (a real driver commits close to the junction, not a block
  away); the default is derived from the loaded map's own block size, so
  it re-derives itself sensibly on a different city rather than carrying
  a stale value over.
- **Routing** — A*, Dijkstra or BFS, applied to every vehicle spawned
  from here on (to retarget an existing vehicle's algorithm, see
  `Modify Vehicles` in §5). See
  [docs/experiments_report.md](experiments_report.md) for how these three
  actually compare under load.
- **Random events** — accidents, closures, congestion spikes and signal
  faults firing on their own every 18–45s. Turn this off for a
  controlled, repeatable run.
- **Auto-spawn traffic** — keep spawning up to the cap above; turn it off
  to stop new arrivals and watch the existing fleet clear out.

## 4. Watching traffic

Vehicles come in four types — car, bus, emergency, motorbike — drawn with
their own sprite and (for cars) one of four colours picked deterministically
from the vehicle's id, so the same city looks the same on every reload.
Click any vehicle to open its detail card and highlight its planned route
on the map; click the `×` on that card, or click empty ground, to clear the
selection.

Traffic lights show red/amber/green per the OSM data a city was imported
with (§7) or per lights you place yourself (§5); a junction with no light
falls back to give-way arbitration per §3's "Give way at" setting.

## 5. Editing the map live

All under `Map`. Each tool stays active — keep clicking to place more —
until you press `Esc` or pick a different tool:

- **Add Building** — pick House, Company or Warehouse, then click the map.
- **Add Road** — click two points: an existing junction, a point partway
  along a road (splits it there), or empty ground (creates a new
  junction). A road to a brand-new dead end is always two-way. Set speed
  (km/h) and lane count first, in the sidebar.
- **Add Vehicle** — pick Car, Bus or Emergency, a count, then confirm to
  spawn them at random valid start points.
- **Add Traffic Lights** — click a junction to signalize it: its
  approaches get stop lines a few metres before the box, and opposing
  directions (N+S vs E+W) alternate green → amber → red. Set the green
  phase length first.
- **Delete Roads / Buildings / Traffic Lights / Vehicles / Decoration** —
  click the thing on the map to remove it. Deleting a road despawns
  vehicles on it and reroutes everyone else still headed that way.
  "Remove ALL Vehicles" is a single button inside Delete Vehicles.
- **Modify Vehicles** — click a vehicle, then either change its routing
  algorithm or send it to a newly-picked destination.
- **Modify Roads** — click a road to retune its speed limit or toggle it
  closed.
- **Modify Intersections** — click a junction to inspect it, toggle
  whether it's signalized, or delete the junction entirely.

## 6. Incidents

`Incident` menu — each one targets a random road/vehicle unless you've
selected one first, and each dispatches (or affects) something specific:

- **Fire** — places an accident on the target road and dispatches two fire
  units.
- **Medical Emergency** — dispatches an ambulance (ignores red lights,
  cuts through congestion) to a random destination.
- **Robbery** — causes a local jam on the target road and sends two police
  units.
- **Smash Road** — closes the target road for a chosen duration (seconds);
  traffic reroutes around it until it reopens.

## 7. Importing a real city from OpenStreetMap

`Map → Sync from OpenStreetMap…` opens an area picker over the running
simulation — the current city keeps going underneath, so cancelling costs
nothing. Drag to select an area, shift+drag to pan, scroll to zoom, name
it, then **Lock map** to import.

Real `highway=traffic_signals` data comes in with the import: a junction
only gets a light if OSM actually has one there, on the actual arm OSM put
it on — imported cities don't invent signals the real place doesn't have.

## 8. Saving and loading

- `Files → Open` lists maps already in `data/`; loading one replaces the
  running city (every current vehicle despawns).
- `Files → Save` writes over the map you're currently running as
  `autosave`; `Files → Save As` writes a new file under `data/<name>.geojson`
  — both include every road, building and light you've added live, not
  just what the map started with.
- `Map → Map Details` shows the currently loaded city's stats (node/road/
  building counts and similar).

## 9. Scrubbing through history

The bar at the bottom of the screen. The handle starts on the right —
**live, right now** — and dragging it left moves back through the last 24h
of this simulator run (a fresh launch always starts this window empty; see
[architecture.md](architecture.md) if you're curious why).

The label next to the handle is the simulator's **own** clock, not your
system clock: it starts at the real time the simulator launched and then
counts forward using simulated time, which is why it can drift ahead of or
behind the actual wall clock if you've been running above or below 1x
speed (§3). A city that's been running at 5x for ten real minutes will show
a clock reading roughly fifty minutes ahead of when you started it.

While scrubbed into the past:

- The whole screen goes **view-only** — a banner reads "VIEWING HISTORY"
  and every editing tool, incident, and click-to-select is disabled. The
  live simulation is **not** paused; it keeps running underneath for
  everyone else, and for you the moment you come back.
- **Jump to Live** snaps straight back to now and re-enables everything.

## 10. Diagnostics

`View` menu:

- **Statistics** — trips completed, vehicles spawned/active, route
  recomputations, average speed, lane changes, gridlock breaks/rescues,
  random events fired, a per-algorithm table (runs / µs / nodes explored /
  failures), and a rolling congestion chart.
- **Event Log** — a running feed of what's happened (spawns, incidents,
  edits, reroutes).
- **FPS Meter** — rendering performance, useful mainly on a very large
  imported city.

## 11. Using the API directly

Everything the UI does goes through `GET`/`POST` endpoints under `/api/...`
— see the [README](../README.md#the-rest-api) for the full list and
`web_frontend/js/api.js` for the authoritative client-side wrapper. A few
to try directly:

```
curl "http://localhost:8080/api/state"                    # full live snapshot
curl "http://localhost:8080/api/map"                       # node/road ids -- pick two for the next line
curl "http://localhost:8080/api/spawn?from=<node id>&to=<node id>&type=car"
curl "http://localhost:8080/api/history/range"              # how far back playback goes
curl "http://localhost:8080/api/history/at?ago=120"          # a snapshot from 2 minutes ago
```

## 12. Running the routing benchmarks

Not part of the running app, but built alongside it. `route_bench.cpp`
compares BFS/Dijkstra/A* offline on a static map;
`tools/run_experiments.py` drives the same three algorithms through a
*running* simulation under several traffic scenarios and writes charts plus
a report. See
[docs/experiments_report.md](experiments_report.md) for the results and
what they mean, or run `python tools/run_experiments.py --help` for the
tool's own options (`--smoke` for a one-run sanity check before committing
to the full sweep).
