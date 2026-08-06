# Module 10 — Synthesis: One Vehicle, End to End

Every earlier module studied one file group in isolation. This one does the
opposite: it follows a single event — **a new car is spawned, routed, driven
through an intersection, and eventually shown in the browser** — straight
through every module, in the order the code actually executes it. If you've
read modules 01–09, this should read like a rerun with the pieces now
familiar; if you've only skimmed them, the cross-links below point back to
where each mechanism is explained line by line.

---

## The request, phase by phase

### 1. The map exists before anything else can

Nothing in this trace is possible until a `TrafficGraph` exists. At startup,
`main.cpp` (see [`learn/09-server-and-entrypoint.md`](09-server-and-entrypoint.md))
loads a map through `MapLoader` (see
[`learn/08-supporting-systems.md`](08-supporting-systems.md)), which parses
either this project's native GeoJSON format or an OSM/Overpass export into a
populated `TrafficGraph` — `Intersection` nodes, `Road` edges, `Building`s
anchored to it, decorative `Zone`s alongside it (see
[`learn/02-core-graph-model.md`](02-core-graph-model.md)). `TrafficLight`s are
attached to intersections and their phase offsets are derived deterministically
from intersection IDs (see
[`learn/03-traffic-lights.md`](03-traffic-lights.md)) — so two runs of the same
map produce the same signal timings without persisting extra state. This
graph is handed to `TrafficSimulator::instance()`, the Meyer's singleton
(function-local `static TrafficSimulator sim;`) that owns "the one and only
running city" (see [`learn/07-simulation-core.md`](07-simulation-core.md)).

### 2. Spawning: a vehicle is born already wired into two systems

When `TrafficSimulator` spawns a vehicle, two things happen that make the
rest of this trace possible:

- The `Vehicle` (a `Car`, `Bus`, `EmergencyVehicle`, or `Motorbike` — see
  [`learn/05-vehicles.md`](05-vehicles.md)) is constructed with a
  `std::unique_ptr<PathFinding>` router chosen by `make_router()`, defaulting
  to `Dijkstra` if the requested algorithm name isn't recognized (see
  [`learn/04-routing.md`](04-routing.md)).
- The vehicle is **attached as an Observer** of `TrafficController`, the
  Observer *subject* (see
  [`learn/06-events-observer.md`](06-events-observer.md)) — this is what lets
  the vehicle later react to a road closure or an accident it didn't cause,
  without `TrafficSimulator` having to manually notify every vehicle by hand.

This is the first place two modules that look unrelated in isolation
(routing and the Observer pattern) turn out to be wired together at
construction time: a vehicle is simultaneously "a thing with a route" and "a
thing that listens for events," and neither role knows about the other.

### 3. Routing: picking a path is a polymorphic call, not a branch

The vehicle calls `run_router()`, which calls `router_->find_path(...)`
**unconditionally** — the call site never asks "am I holding a BFS or an
A*?" (see [`learn/04-routing.md`](04-routing.md)). This is the Strategy
Pattern doing real work: `PathFinding.hpp` is header-only (no `.cpp` — there's
nothing to define out of line for a pure interface), and `BFS`, `Dijkstra`,
and `AStar` each override `find_path` with a genuinely different data
structure (`std::queue` for BFS, a min-heap `std::priority_queue` for the
other two) and, for BFS, a genuinely different *correctness contract* —
fewest edges, not lowest cost. `AStar`'s heuristic is straight-line Euclidean
distance to the goal, provably admissible because road length is always
≥ straight-line distance between its endpoints. Whichever strategy runs, the
result is a sequence of `Road`/`Intersection` graph elements the vehicle will
now try to drive.

### 4. The tick loop: five phases, one thread, every vehicle

From here on, the vehicle's life is governed entirely by
`TrafficSimulator::tick()`, called from a dedicated tick thread roughly 20
times a second (see
[`learn/07-simulation-core.md`](07-simulation-core.md) and
[`learn/09-server-and-entrypoint.md`](09-server-and-entrypoint.md)). Each
call holds one mutex (`mtx_`) for its entire duration and runs five phases,
strictly in order, as five separate `for` loops over `vehicles_` — **not** a
thread pool, **not** a parallel `for_each`. This is the literal, structural
form of the "single-threaded tick loop" limitation: it isn't a metaphor, it's
five sequential scans of one vector on one thread, every tick, and the file's
own header comment explains *why* the order is fixed — every decision that
needs a fleet-wide view (signals, MOBIL lane changes) has to happen before
anyone actually moves, and every leader vehicle is snapshotted at the start
of phase 2 so that vehicle #2's decision this tick can't depend on whether
vehicle #1 already moved.

1. **Events and lane changes.** Signals advance (a pure function of
   `sim_time`, not a mutated field — see
   [`learn/03-traffic-lights.md`](03-traffic-lights.md)); scheduled
   `TrafficEvent`s fire through `TrafficController::trigger_event`, which
   applies the event to the graph, *then* calls `notifyAll()` so every
   attached vehicle-Observer hears about it only after the graph already
   reflects it (see
   [`learn/06-events-observer.md`](06-events-observer.md)); and
   `JunctionArbiter::solve_lane_changes()` evaluates MOBIL incentive/safety
   checks for any vehicle eligible to change lanes, gated by a hard minimum
   gap (`kLaneChangeMinGap`) to stop cars from merging into an already-occupied
   slot (see [`learn/05-vehicles.md`](05-vehicles.md) and
   [`learn/07-simulation-core.md`](07-simulation-core.md)).
2. **Leads and physics.** Every vehicle's leader (the car ahead of it, if
   any) is resolved and cached first; then every vehicle computes an IDM
   (Intelligent Driver Model) acceleration from that leader, and
   `JunctionArbiter::solve_junctions()` runs a box-occupancy check — "is
   anyone from a different street within ~2 meters of this node" — before a
   vehicle already at an intersection is allowed to proceed.
3. **Cleanup and spawn.** Vehicles that reached their destination are
   removed (and, per step 2 above, that removal detaches them as an
   Observer too); `JunctionArbiter::break_cycles()` runs a real
   wait-for-graph cycle detection over vehicles currently blocked on each
   other, but only once every two seconds; `solve_gridlock()` — described in
   its own source comment as "the escape hatch of last resort" — forcibly
   frees vehicles stuck past a stall threshold; new vehicles are spawned to
   keep traffic flowing.
4. **Statistics.** `StatisticsManager` samples the network state for the
   dashboard, including per-algorithm routing performance — the empirical
   feedback loop for the Strategy Pattern from phase 3-of-the-trace, above
   (see [`learn/08-supporting-systems.md`](08-supporting-systems.md)).
5. **History capture.** Once per *real* second (not simulated second — it's
   keyed to wall-clock time so playback speed is independent of simulation
   speed), `HistoryStore` persists a frame to disk as a baseline-plus-delta
   binary encoding, enabling later playback.

### 5. Getting from the tick loop to the browser

None of the above is visible to a user until it crosses the network boundary,
and this is where the project's naming is genuinely misleading: the class
called `WebSocketServer` is not a WebSocket server. Its own source comment
admits it's named that "for historical reasons" — it's a hand-rolled `HttpServer`
speaking plain HTTP, one detached `std::thread` per accepted connection, no
handshake or framing. The browser (`web_frontend/js/api.js`) confirms this
independently: every call is a `fetch(...)`, never `new WebSocket(...)` (see
[`learn/09-server-and-entrypoint.md`](09-server-and-entrypoint.md)). A separate
publisher thread refreshes a JSON snapshot (`/api/state`, ~10Hz) and the tick
thread itself refreshes a binary snapshot (`/api/stateBin`, every tick, ~20Hz)
behind two *different* mutexes from the simulation's own `mtx_`, specifically
so an HTTP request reading the last-published frame never has to wait behind
an in-progress tick. The vehicle you spawned in step 2 becomes visible in the
browser only when it's captured into one of these snapshots and the frontend's
next poll happens to land after that.

---

## Revisiting the four known limitations, now with the full path in view

**Vehicles are locked to discrete lanes.** `Vehicle::lane` is a plain `int`.
The `lateral_offset`/`target_lateral_offset` fields that look like a
continuous-position mechanism are purely cosmetic — they animate the sprite
sliding across during a lane change, but no gap check, no collision check,
and no MOBIL decision ever reads them. Every safety-relevant comparison in
phase 1 and phase 2 of the tick (above) is an exact integer equality on
`lane`. This is why a vehicle can never occupy "lane 1.5" — there is no
representation for it anywhere in the data model, not just a missing
feature in the physics.

**Junction overlap can happen.** Two independent gaps, both surfaced in
module 07: `JunctionArbiter::solve_lane_changes()`'s own comment admits its
leader/follower search is strictly ahead/behind on the *same* lane, so two
vehicles at an identical `position_on_road` — which can arise from the
`kLaneChangeMinGap` gate not covering every code path that sets `lane`
directly — are invisible to each other. Separately, `solve_junctions()`'s
occupancy check is a flat-radius "is anyone nearby" test with no model of
actual turning-movement geometry (which lanes actually cross depends on the
turn each vehicle is making, not just proximity), so it can be both too
strict and too permissive depending on the exact geometry at a given
junction.

**Gridlock is handled reactively, not prevented.** Both mechanisms that deal
with it — `break_cycles()` and `solve_gridlock()` — run in phase 3, *after*
vehicles are already stopped and already waiting on each other. Nothing in
phase 1 (where lane-change and signal decisions are made) asks "would
admitting this vehicle to this junction create a future cycle?" The
detection is real (`break_cycles()` does genuine cycle detection over a
wait-for graph), but it's detection-and-recovery, not admission control —
exactly the distinction the word "reactive" is doing work to convey.

**The tick loop is single-threaded.** Concretely: `tick()`'s five phases are
five sequential loops over `vehicles_`, on one thread, holding one mutex for
the whole call. The `Threads::Threads` link dependency you saw all the way
back in [`learn/01-orientation.md`](01-orientation.md) has nothing to do with
parallelizing this loop — it's for the HTTP accept-thread and the publisher
thread described in step 5, above. Those threads read simulation *snapshots*
under their own separate mutexes; they never touch `vehicles_` or call into
`tick()`'s logic directly. The simulation's actual per-tick work — routing
decisions, physics, junction arbitration — is exactly as sequential as a
single-core program would be.

---

## Check yourself

- Trace what happens if a `RoadClosureEvent` fires (phase 1) on the road a
  vehicle's cached route currently uses. Which module's mechanism notices,
  and how many ticks can pass before the vehicle actually reacts?
- Why does the tick loop snapshot every leader at the start of phase 2 instead
  of just calling `leader_of(vehicle)` fresh inside each vehicle's own
  update? What would change about the simulation's behavior — not just its
  performance — if it didn't?
- The HTTP server and the tick loop run on different threads but neither
  blocks the other for long. Name the two mutexes that make that possible,
  and what each one actually guards.
- If you were asked to fix *only* the junction-overlap limitation with the
  least code change, would you extend `solve_lane_changes()`'s gap check or
  `solve_junctions()`'s occupancy check — and why does the geometry argument
  above suggest one is a bigger job than the other?
