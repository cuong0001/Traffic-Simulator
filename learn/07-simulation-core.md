# Module 07 — The Simulation Core

**Files covered:**
- [`cpp_backend/include/simulation/TrafficSimulator.hpp`](../cpp_backend/include/simulation/TrafficSimulator.hpp)
- [`cpp_backend/src/simulation/TrafficSimulator.cpp`](../cpp_backend/src/simulation/TrafficSimulator.cpp)
- [`cpp_backend/include/simulation/JunctionArbiter.hpp`](../cpp_backend/include/simulation/JunctionArbiter.hpp)
- [`cpp_backend/src/simulation/JunctionArbiter.cpp`](../cpp_backend/src/simulation/JunctionArbiter.cpp)

Everything before this module was a piece sitting still: a graph that doesn't move,
a vehicle that doesn't know the fleet exists, an Observer subject that only fires
when told to. `TrafficSimulator` is the piece that makes time pass. It owns the one
and only running city, drives it forward `dt` seconds at a time, and hands the parts
of that work that need a fleet-wide view — right of way, lane changes, deadlock
relief — to `JunctionArbiter`. This is also where two of the three "known limitations"
named in the project README actually live in code, not just in a bullet list: junction
overlap and reactive (rather than preventive) gridlock handling. The third — the
single-threaded tick loop — is a structural fact about this file's control flow, and
you'll see exactly where it is and is not true.

---

## `cpp_backend/include/simulation/TrafficSimulator.hpp`

### Lines 1–19: the file banner — five phases, one world, two threads

```cpp
#pragma once
/*
TrafficSimulator.hpp

- The simulation engine. Singleton: exactly one world, shared by the
tick thread and the HTTP server thread, so every public method locks.

- The tick is split into five phases that always run in this order:
    1. events and lane changes   advance signals and events, run MOBIL
    2. leads and physics         resolve every leader, then move everyone
    3. cleanup and spawn         reap arrivals, rescue deadlocks, top up
    4. statistics                sample the network for the charts
    5. history capture           persist one frame every REAL second, for playback

- The order matters. A Vehicle can only see its own road, so all the decisions
that need a global view are made in phase 1 and 2 *before* anything moves,
and every leader is snapshotted first -- otherwise the first vehicle to move
would change what the second one sees inside the same tick.
*/
```

**Meaning:** `#pragma once` is the usual non-standard-but-universal include guard.
The block comment is doing something this codebase does consistently and that you
should trust: it states the file's contract in prose before a single declaration
appears. Two claims matter here. First, "Singleton: exactly one world, shared by the
tick thread and the HTTP server thread" — there are at least two threads touching this
object, and the *simulation state itself* (`vehicles_`, `graph_`, `sim_time_`, …) is
one mutable structure both of them can reach. Second, the tick is not one big loop
over vehicles; it's five named phases run in a fixed order, and the comment tells you
*why* the order is fixed: phase 1 and 2 make every decision that needs to see other
vehicles before phase 2 moves anyone, and leaders are captured as a snapshot rather
than looked up live so that vehicle #2's decision doesn't depend on whether vehicle
#1 has already moved this tick.

**Why:** this is the same problem you'd hit in any discrete-time simulation of
interacting agents — if you update agents one at a time and let later agents see
already-updated earlier agents, the simulation becomes order-dependent: swap the
iteration order and you get different physics. The fix here is "resolve, then apply":
decide everyone's leader and every yield decision against the *pre-tick* world, then
apply all the motion. That's exactly what `tick_leads_and_physics()` does later —
build a `leaders` vector for the whole fleet first, then loop again to call
`advance()` on each vehicle using that frozen snapshot.

**Usage:** every phase function you'll meet in the `.cpp` file exists because this
comment promises it does, in this order. If you're hunting a bug where a vehicle's
decision seems to depend on its index in `vehicles_`, this ordering guarantee is the
first thing to check — and the "leaders snapshotted first" line is the reason it
*shouldn't* be broken, unless something reads live state mid-phase-2 instead of the
snapshot.

**Check yourself:**
- Why does phase 1 have to run for *every* vehicle before phase 2 moves *any* vehicle, rather than interleaving "decide, then move" per vehicle?
- What would you observe in the simulation if `tick_leads_and_physics()` looked up each vehicle's leader lazily, right before calling `advance()`, instead of building the `leaders` vector up front?

---

### Lines 21–33 and 74–76: includes and forward declarations

```cpp
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <unordered_map>

#include "core/Road.hpp"
...
class TrafficGraph;
class Vehicle;
```

**Meaning:** the standard-library includes read like a table of contents for what
this class does: `<mutex>` and `<atomic>` for the multi-thread coordination the
banner comment promised, `<thread>` for spawning the publisher thread, `<random>`
for `auto_spawn()`'s dice rolls, `<memory>` for `unique_ptr`/`shared_ptr` ownership.
`core/Road.hpp` is included by value (not forward-declared) because the binary
snapshot structs below embed `Road::base_index` semantics directly. `TrafficGraph`
and `Vehicle` are forward-declared, not included — the header only ever holds a raw
pointer (`TrafficGraph* graph_`) and vector-of-`unique_ptr` (`vehicles_`), neither of
which needs the full class definition to compile.

**Why:** forward declarations here keep `TrafficSimulator.hpp` from pulling in the
entire vehicle/graph header graph — this header is included by `main.cpp`, the
WebSocket server, and effectively every test executable, so keeping its include
surface small keeps rebuild times down.

**Usage:** the corresponding `#include "core/Graph.hpp"`, `#include "vehicles/Vehicle.hpp"`,
etc. show up in the `.cpp` file instead, where the full type is actually needed to
call methods on it.

**Check yourself:**
- Why is `core/Road.hpp` a full include here while `TrafficGraph` and `Vehicle` are only forward-declared?

---

### Lines 34–66: `VehicleSnapshotBin` and `BinarySnapshotHeader` — the wire format

```cpp
#pragma pack(push, 1)
struct VehicleSnapshotBin {
    uint32_t num_id;
    uint32_t base_index;
    float pos;
    float speed;
    float lateral_offset;
    uint8_t lane;
    uint8_t type;
    uint8_t flags;
};
#pragma pack(pop)
static_assert(sizeof(VehicleSnapshotBin) == 23, "VehicleSnapshotBin layout must be 23 bytes");

#pragma pack(push, 1)
struct BinarySnapshotHeader {
    uint32_t magic;
    uint16_t version;
    uint16_t vehicle_count;
    double sim_time;
};
#pragma pack(pop)
```

**Meaning:** `#pragma pack(push, 1)` tells the compiler to drop its usual struct
padding/alignment for the following struct(s), so every field sits immediately after
the previous one with no gap — the struct's in-memory layout becomes byte-identical
to what gets written to the wire. Every field is a fixed-width type (`uint32_t`,
`float`, `uint8_t`) rather than something platform-dependent like `int` or `long`, so
the layout is portable between the C++ backend and the JavaScript frontend reading
the same bytes with a `DataView`. The `static_assert` is a compile-time guarantee: if
anyone ever adds or reorders a field and the size drifts from the frontend's
expectation, the build fails immediately instead of silently corrupting every vehicle
position on screen. The comment above the struct (not reproduced above, see the
actual file) explicitly says this is road-relative — `road` + `pos` + `lane` + `lat`
— rather than world-space x/y, and explains why: only the frontend's `placeOnRoad()`
knows a road's rendered width and the map's cosmetic scale factor, so computing
world coordinates on the backend risks disagreeing with the frontend about constants
neither side shares.

**Why:** this is a hand-rolled binary protocol instead of JSON because `state_json()`
formats numbers as decimal text — cheap to read but comparatively expensive to
produce and to parse, and wasteful for something published every tick. A `VehicleSnapshotBin`
is 23 bytes of `memcpy`-able POD; building `count` of them is fast enough to redo on
every tick (see `publish_binary_snapshot_locked()` below), where `state_json()`'s
string-building work is deliberately throttled to 10 Hz instead.

**Usage:** `TrafficSimulator::encode_binary_snapshot_locked()` (in the `.cpp`) fills
these structs from live `Vehicle`/`Road` state; the frontend's WebSocket handler
decodes the same byte layout. `magic` (`"TRAF"`) and `version` let the frontend sanity
check the format before trusting the payload.

**Check yourself:**
- Why does `VehicleSnapshotBin` store `base_index` and `lane` instead of a world-space `x`/`y` position?
- What class of bug does the `static_assert` on `sizeof(VehicleSnapshotBin)` prevent, and why would that bug otherwise be silent?

---

### Lines 68–72: the composed subsystems

```cpp
#include "simulation/HistoryStore.hpp"
#include "simulation/JunctionArbiter.hpp"
#include "simulation/MapEditor.hpp"
#include "simulation/TrafficController.hpp"
#include "utils/StatisticsManager.hpp"
```

**Meaning:** five headers, each a class `TrafficSimulator` will hold a member of by
value later: `HistoryStore` (playback), `JunctionArbiter` (the fleet-wide solver
covered in the second half of this module), `MapEditor` (live map edits), `TrafficController`
(the Observer subject from [`learn/06-events-observer.md`](06-events-observer.md)),
and `StatisticsManager` (aggregate stats for the charts).

**Why:** this is composition, not inheritance — `TrafficSimulator` is a coordinator
that *owns* each of these rather than *being* one. Splitting the solver out into
`JunctionArbiter` and the editing logic out into `MapEditor` keeps `TrafficSimulator`
itself from becoming a single class that does routing arbitration, map mutation,
history capture, and HTTP JSON formatting all in one file — each concern gets a class
boundary of its own, which is exactly what `JunctionArbiter.hpp`'s own file banner
says its purpose is.

**Usage:** you'll see `arbiter_`, `map_editor_`, `controller_`, `stats_`, and `history_`
as private members further down — every one of these includes exists to make that
member declaration compile.

**Check yourself:**
- Why does `TrafficSimulator` hold `JunctionArbiter` and `MapEditor` as members instead of, say, `TrafficSimulator` inheriting from them?

---

### Lines 77–80: the class declaration and the Singleton accessor

```cpp
class TrafficSimulator {
public:
    // Singleton access.
    static TrafficSimulator& instance();
```

**Meaning:** `static TrafficSimulator& instance();` is the entire public surface of
the Singleton pattern here. It is a `static` member function — callable as
`TrafficSimulator::instance()` without any existing object — that returns a reference
to *the* instance. There is no `getInstance()`-style raw-pointer variant and no
public constructor (you'll see the constructor is `private` further down). The
comment directly above it, `// Singleton access.`, is the exact comment the project's
own architecture notes point to as confirmation that this class is a Singleton.

**Why (confirming the mechanism):** the declaration alone doesn't tell you *which*
Singleton idiom is used — that's decided by the definition in the `.cpp` file, which
you'll see is:

```cpp
TrafficSimulator& TrafficSimulator::instance() {
    static TrafficSimulator sim;
    return sim;
}
```

This is a **Meyer's Singleton** (function-local `static`), not a heap-allocated
`static TrafficSimulator* instance_ = nullptr;` checked-and-lazily-`new`'d pattern.
The distinction matters: a function-local `static` is guaranteed by the C++11
standard to be initialized exactly once, in a thread-safe way, the first time control
passes through the declaration — the compiler emits the double-checked-locking
machinery for you. A hand-rolled pointer-based Singleton would need its own mutex or
`std::call_once` to be safe under concurrent first access from multiple threads; the
Meyer's form gets that for free from the language, which is exactly what this class
needs given the banner comment's claim that the tick thread and the HTTP server
thread both reach this object. Destruction order is also handled by the language:
`sim` is destroyed at static-deinitialization time, same as any function-local static.

**Usage:** every caller reaches simulation state through `TrafficSimulator::instance()`
— there is no other way to get a `TrafficSimulator&`, because the constructor is
private and copy/move are deleted (see below). `main.cpp` calls `instance().load(...)`
and `instance().start_publishing()` at boot; the WebSocket server's HTTP handlers call
`instance().tick(dt)`, `instance().spawn_vehicle(...)`, `instance().state_json()`, and
so on, for every incoming request. Every one of those call sites gets the *same*
object — that's the invariant the Singleton protects: "exactly one world, shared by
the whole process." If two `TrafficSimulator` instances could exist, the tick loop
could be advancing one city while an HTTP handler reads or mutates a different one,
silently — requests and playback would disagree about what's actually happening, and
nothing would announce the split. The Singleton makes that scenario a compile-time
impossibility rather than a bug you'd have to find at runtime.

**Check yourself:**
- What specifically does the C++ standard guarantee about a function-local `static` variable's initialization that a hand-rolled `static TrafficSimulator* ptr = nullptr; if (!ptr) ptr = new TrafficSimulator();` pattern does *not* get for free?
- If `TrafficSimulator` allowed public construction (no private constructor, no deleted copy), what specific invariant described above would break, and what symptom would you actually see in the running app?

---

### Lines 82–98: `load()`, `tick()`, `spawn_vehicle()`, `trigger_event()`

```cpp
void load(const std::string& map_path);
void tick(double dt);
std::string spawn_vehicle(const std::string& from, const std::string& to,
                          const std::string& type);
std::string trigger_event(const std::string& type, const std::string& target,
                          double duration);
```

**Meaning:** `load()` replaces the running world with a freshly parsed map. `tick(double dt)`
is the one method that advances simulated time — `dt` is a real-seconds duration, later
scaled by the speed factor inside the method. `spawn_vehicle()` is a factory call — see
[`learn/05-vehicles.md`](05-vehicles.md) for `VehicleFactory` — that returns the new
vehicle's id or `""` on failure (unreachable pair, or the entry lane already occupied).
`trigger_event()` forwards to the Observer subject; see [`learn/06-events-observer.md`](06-events-observer.md)
for what `TrafficController::trigger_event` actually does.

**Why:** all four are declared to return by value / take by `const&` rather than
exposing any internal pointer — callers (HTTP handlers) never get a raw reference
into `vehicles_` or `graph_` that could dangle after the next `tick()` mutates the
container.

**Usage:** these four, together with `state_json()`/`binary_snapshot()`, are the
methods the WebSocket/HTTP server layer (see `learn/09-server-and-entrypoint.md`)
calls directly per request. `tick()` specifically is called from a dedicated tick
thread started in `main()` — not from an HTTP handler — because it has to run on a
fixed cadence regardless of whether any request is in flight.

**Check yourself:**
- Why does `spawn_vehicle()` return an empty string on failure instead of throwing an exception?

---

### Lines 100–109: playback controls and traffic-generation tuning

```cpp
void set_paused(bool paused);
void set_speed(double factor);
double speed() const { return speed_; }
void set_auto_spawn(bool on);
void set_random_events(bool on);

void set_target_vehicles(int n);   // 0 = unlimited
void set_spawn_rate(double per_second);
```

**Meaning:** a batch of simple setters/one getter controlling whether the tick loop
does anything (`paused_`), how fast simulated time passes relative to real time
(`speed_`), whether the city keeps spawning traffic on its own (`auto_spawn_`),
whether random events fire (`controller_.set_random_events`), and the auto-spawn
target population / rate. `speed()` is the only inline getter in this batch — cheap
enough, and read often enough (every `state_json()` call), to not bother with an
out-of-line definition.

**Why:** these exist because the web UI's control panel needs to change simulation
behavior live, without restarting the process or reloading the map. `set_target_vehicles(0)`
being documented as "unlimited" rather than "zero vehicles" is a small but real API
design choice — it reuses an `int` parameter's otherwise-meaningless zero value
instead of adding a separate boolean flag.

**Usage:** all of these are thin wrappers in the `.cpp` that take `mtx_` and set one
field — you'll see the pattern repeated for nearly every public setter in this class.

**Check yourself:**
- Why is `set_target_vehicles` and `set_spawn_rate` separated as two independent knobs rather than one "spawn `n` vehicles then stop" call?

---

### Lines 111–129: the junction-approach knob

```cpp
// ---- the give-way distance -------------------------------------------
// How close to an unsignalized junction a vehicle has to be before it
// starts working out whether to give way, in METRES measured from the far
// end of the road it is on. THIS IS THE KNOB. ...
void set_junction_approach(double metres);
double junction_approach() const;
```

**Meaning:** this is a single tunable distance (in metres) that controls when a
vehicle starts negotiating right-of-way at an unsignalized junction, forwarded
straight through to `JunctionArbiter` (you'll meet `junction_approach_` again in that
class). The comment explains the tuning story in detail: a distance calibrated for a
synthetic map with huge (240 m) blocks is wrong for a real OpenStreetMap extract with
~27 m median blocks — too large a value means a car is "negotiating" for its entire
trip down a city block, so the value is *derived from the loaded map* rather than
hardcoded (see `default_junction_approach()` later), clamped to `[7, 60]`, and
re-derived every time a new map loads.

**Why:** a fixed constant that happens to be right for one map's block size is wrong
for another's — this is a case where the "right" tuning value is a property of the
data (the city), not a property of the code, so the code computes it from the data
instead of shipping one magic number.

**Usage:** live-tunable through `GET /api/junctionApproach?value=<metres>` and
reported back in `/api/state`, so a person can watch traffic and dial the value in
rather than rebuild the binary to test a guess. `JunctionArbiter::solve_junctions()`
is the actual consumer, gating whether a vehicle is close enough to a junction to
start evaluating right of way at all.

**Check yourself:**
- Why is the give-way distance measured "from the far end of the road" rather than as an absolute world-space radius around the junction node?
- What concrete failure mode does the comment describe happening on a real OSM extract if this value were left at the reference implementation's synthetic-map constant?

---

### Lines 130–150: routing/cost knobs, and the two snapshot methods

```cpp
bool set_algorithm(const std::string& name);
void set_cost_mode(CostMode mode);

void encode_binary_snapshot(std::vector<uint8_t>& buffer);
std::shared_ptr<const std::vector<uint8_t>> binary_snapshot();
```

**Meaning:** `set_algorithm` picks which `PathFinding` strategy (`"astar"|"dijkstra"|"bfs"`,
see [`learn/04-routing.md`]) new vehicles use; `set_cost_mode` picks what a
route minimizes (distance vs. time). `encode_binary_snapshot()` locks, rebuilds, and
fills a caller-supplied buffer on the spot — the comment says this is fine for a
direct/test caller but the HTTP path should prefer `binary_snapshot()` instead.
`binary_snapshot()` returns a `shared_ptr` to an *already-published* buffer, refreshed
once per tick (not on `state_snapshot()`'s slower timer), without the caller ever
taking the simulation mutex.

**Why:** the comment is explicit about the motivation: positions are the one thing
that actually needs tick-rate (not just 10 Hz) freshness, and a `VehicleSnapshotBin`
is cheap enough (a handful of POD field copies) to afford rebuilding every tick, unlike
`state_json()`'s string formatting. The gap between the two refresh rates is called
out as the specific thing that made `/api/state` feel ~50 ms stale even after the
binary format existed — i.e. this is a documented, deliberate fix for a previously
observed staleness bug, not a speculative optimization.

**Usage:** `tick()` itself calls `publish_binary_snapshot_locked()` at the end of
every phase-5 pass (see the `.cpp` walk-through below); the WebSocket layer calls the
public `binary_snapshot()` to get the latest published copy for outgoing frames.

**Check yourself:**
- Why does `encode_binary_snapshot()` (public, locking, rebuild-on-call) exist at all if `binary_snapshot()` is the preferred HTTP path — what's it actually for?

---

### Lines 152–172: vehicle operations and live map editing

```cpp
bool remove_vehicle(const std::string& id);
int clear_vehicles();
bool retarget_vehicle(const std::string& id, const std::string& destination);
bool set_vehicle_algorithm(const std::string& id, const std::string& algo);
std::string vehicle_detail_json(const std::string& id);

std::string add_junction_at(double x, double y);
std::string split_road_near(double x, double y);
std::string add_road_between(const std::string& from, const std::string& to,
                             double speed_kmh, int lanes, bool oneway);
bool remove_road(const std::string& id);
bool remove_intersection(const std::string& id);
bool update_road(const std::string& id, double speed_kmh, bool closed);
bool set_light(const std::string& node_id, bool on, double green);
std::string add_building_at(double x, double y, const std::string& kind,
                            const std::string& name, double size);
bool remove_building(const std::string& name);
```

**Meaning:** two batches of API surface. The first operates on individual vehicles
already in the simulation (remove one, clear all, change a destination or algorithm,
fetch one vehicle's detail JSON for the frontend's selection card). The second is
*live* map editing — add or remove a junction, road, or building while the simulation
keeps running, plus toggling a traffic light. Every one of these is declared to
return either a new element's id (`""` on failure) or a `bool` success flag — no
exceptions cross this API boundary.

**Why:** the doc comment above the map-editing block ("Vehicles on a removed element
are despawned and routed vehicles are asked to replan") tells you these aren't
passive data edits — removing a road mid-simulation has to cascade into vehicle state,
which is why this can't just be a `MapLoader`-style batch reload; it has to coordinate
with `vehicles_` and `controller_` too. That's also why `TrafficSimulator` doesn't
implement these itself — `map_editor_` does, and `TrafficSimulator`'s methods here are
thin locking forwarders (confirmed later in the `.cpp`).

**Usage:** each of these backs one web-editor UI action — dragging a new junction
onto the map, drawing a road, deleting a building. All of it happens through the same
Singleton instance the tick loop is running against, under the same `mtx_`, so an edit
can never race a `tick()` call.

**Check yourself:**
- Why do the map-editing methods return the new id as a `std::string` rather than, say, a pointer to the new `Road`/`Intersection` object?

---

### Lines 179–210: `ImportResult`, `import_map()`, and the two published-state accessors

```cpp
struct ImportResult {
    bool ok = false;
    std::string message;
};
ImportResult import_map(const std::string& name, const std::string& geojson);

TrafficGraph* graph() const { return graph_; }

std::string state_json();
std::shared_ptr<const std::string> state_snapshot();

void start_publishing();
std::string map_json();
```

**Meaning:** `ImportResult` is a small struct rather than a single return type,
because — as the comment says — "the map loaded AND it has a problem worth
reporting" are two independent facts, not mutually exclusive outcomes a single string
or bool could encode cleanly. `import_map()` accepts a GeoJSON document that arrived
over HTTP (a user-picked OpenStreetMap area), and — per its comment — parses it into
a *detached* graph first, so a malformed import can't corrupt the running city; only
a successful parse gets written to disk and loaded for real. `graph()` is a raw,
non-owning accessor to the live graph, used elsewhere in the codebase by code that
needs read access without going through every accessor method. `state_json()` builds
the full JSON state fresh on every call (locking `mtx_`); `state_snapshot()` returns
the last *published* copy without locking `mtx_` at all — same "publish on a schedule,
read the published copy" pattern the binary snapshot uses, but on a slower 100 ms
timer via `publish_loop()`, started once by `start_publishing()`.

**Why:** the block comment above `state_snapshot()` explains the motivation
concretely — `/api/state` used to serialize the whole world *while holding `mtx_`*,
so every browser poll stalled a simulation tick, and the UI polls twice a second per
open tab, so six open tabs meant the tick loop was being blocked twelve times a
second for work that produces an *identical* result regardless of how many tabs
asked. Building it once on a schedule and handing out a `shared_ptr<const std::string>`
means readers never block the tick thread and never block each other.

**Usage:** `start_publishing()` is called exactly once from `main()`, per its own
comment ("Called once from main(). Also loads the history store's on-disk index —
both are 'once at process boot' work"). The HTTP layer's `/api/state` handler should
call `state_snapshot()`, not `state_json()`, for exactly the reason above — though
`state_json()` remains public and is still used directly by callers (e.g. `history_state_at_json`
internally, or tests) that need a guaranteed-fresh read regardless of the publish
cadence.

**Check yourself:**
- Why does `import_map()` parse into a *detached* `TrafficGraph` first instead of parsing directly into the live `graph_`?
- Concretely, what would happen to `/api/state` latency under six open browser tabs if `state_snapshot()` didn't exist and every tab called `state_json()` directly?

---

### Lines 212–214: history playback JSON

```cpp
std::string history_range_json();
std::string history_state_at_json(double seconds_ago);
```

**Meaning:** `history_range_json()` reports what window of history is available (oldest/newest
capture, in real seconds ago); `history_state_at_json(seconds_ago)` reconstructs a full
state-JSON-shaped payload from a captured `HistoryFrame` at a point in the past, for
the scrubber UI.

**Why:** playback is a *separate* data path from live state — it reads from `HistoryStore`,
not from `vehicles_`/`graph_` directly, because the vehicles that existed 10 minutes
ago may no longer exist in `vehicles_` at all.

**Usage:** backs the timeline scrubber in the frontend, letting a user drag backward
through up to 24 real hours of captured frames (see `tick_history_capture()` in the
`.cpp` for how those frames get written).

**Check yourself:**
- Why can't `history_state_at_json()` just filter `vehicles_` by some "existed at time T" flag instead of reading from a separate `HistoryStore`?

---

### Lines 216–222: the private constructor, destructor, and `clear()`

```cpp
private:
    TrafficSimulator() = default;
    ~TrafficSimulator();
    TrafficSimulator(const TrafficSimulator&) = delete;
    TrafficSimulator& operator=(const TrafficSimulator&) = delete;

    void clear(); // free graph - vehicles - events
```

**Meaning:** this is the other half of the Singleton mechanism confirmed above. The
default constructor is `private` — nothing outside this class (and the friend-like
access `instance()`'s function-local `static` has as a member function) can construct
a `TrafficSimulator`. Copy constructor and copy-assignment are explicitly `= delete`d
— the compiler will refuse to compile any code that tries to copy a `TrafficSimulator`,
including copies made implicitly (e.g. passing one by value). Move construction/assignment
are *not* declared here, which under the usual C++ rules means they're implicitly
deleted too, since a user-declared destructor suppresses the implicitly-generated move
operations. `clear()` is a private helper that frees the graph, vehicles, and events —
called both from the destructor and from `load()` before installing a new map.

**Why:** deleting copy (and, transitively, move) is what makes "exactly one instance"
actually enforceable rather than just conventional — without the `= delete`s, nothing
would stop `TrafficSimulator copy = TrafficSimulator::instance();` from compiling and
silently producing a second, independent world with its own `vehicles_`/`graph_`
that immediately diverges from the real one. A private constructor prevents `TrafficSimulator x;`
directly; deleted copy/move prevents cloning the one instance that does exist.
Together they're the full enforcement mechanism behind the Meyer's Singleton pattern
— the function-local `static` alone only guarantees lazy, thread-safe *creation*; it's
these four lines that guarantee *uniqueness*.

**Usage:** if you ever see a compile error trying to pass a `TrafficSimulator` by
value, or to store one in a `std::vector<TrafficSimulator>`, this block is why — and
it should stay that way.

**Check yourself:**
- If only the copy constructor were deleted but copy-assignment were left implicit, what code would still compile that shouldn't?
- Why does a user-declared destructor (`~TrafficSimulator()`) suppress the implicitly-generated move constructor/assignment, and why does that suppression matter here?

---

### Lines 224–236: the five tick-phase methods and the locked spawn helper

```cpp
void tick_events_and_lanes(double dt);
void tick_leads_and_physics(double dt);
void tick_cleanup_and_spawn(double dt);
void tick_statistics(double dt);
void tick_history_capture(double dt);

void auto_spawn(double dt);
std::string random_node() const;

std::string spawn_locked(const std::string& from, const std::string& to,
                         const std::string& type);
```

**Meaning:** the five phases named in the file banner comment, declared as private
methods — each takes the already-speed-scaled `dt`. `auto_spawn()` and `random_node()`
support phase 3's "top up" step. `spawn_locked()` is the actual spawn logic; the
public `spawn_vehicle()` you saw earlier is a one-line wrapper that takes `mtx_` and
calls this. The comment on `spawn_locked` is explicit: "Non-locking spawn, called
from tick()/auto_spawn while mtx_ is held" — this function assumes its caller already
holds the lock; calling it without the lock held is undefined behavior with respect
to thread safety, even though nothing in the type system stops you.

**Why:** splitting "acquire lock, then do the work" (`spawn_vehicle`) from "do the
work, assuming the lock is already held" (`spawn_locked`) is what lets `auto_spawn()`
— called from inside `tick_cleanup_and_spawn()`, which runs inside `tick()`, which
already holds `mtx_` — spawn vehicles without either deadlocking on a non-reentrant
`std::mutex` or duplicating the spawn logic. `std::mutex` is not recursive: if `spawn_vehicle()`
(which locks) were called from code that already holds `mtx_`, the thread would
deadlock against itself. The `_locked` naming convention (you'll see it again on
`destroy_vehicle_locked`, `encode_binary_snapshot_locked`, `publish_binary_snapshot_locked`)
is this codebase's way of documenting that precondition in the name itself rather than
relying entirely on a comment.

**Usage:** `tick()` calls the five `tick_*` phases directly, in order, every call.
`tick_cleanup_and_spawn()` calls `auto_spawn()`, which calls `spawn_locked()` (never
the public locking `spawn_vehicle()` — that would deadlock).

**Check yourself:**
- What would happen, concretely, if `auto_spawn()` called the public `spawn_vehicle()` instead of `spawn_locked()`?
- Why is the `_locked` suffix a naming convention here rather than something enforced by the type system (e.g. a wrapper type that only exists while the lock is held)?

---

### Lines 238–252: the core data members — graph, vehicles, and the two composed solvers

```cpp
TrafficGraph* graph_ = nullptr;
StatisticsManager stats_;
TrafficController controller_;
std::vector<std::unique_ptr<Vehicle>> vehicles_;
int next_id_ = 1;

JunctionArbiter arbiter_{graph_, vehicles_, controller_};
MapEditor map_editor_{graph_, vehicles_, controller_, next_id_};
```

**Meaning:** `graph_` is a raw, owning pointer (freed manually in `clear()` — see the
`.cpp`, `delete graph_;`) rather than a `unique_ptr`, because `load()` needs to
construct the *new* graph before tearing down the old one (you'll see this in the
`.cpp`: it fully loads into a local `unique_ptr` first, to validate, before touching
`graph_`). `vehicles_` is `vector<unique_ptr<Vehicle>>` — the simulator owns every
vehicle for its lifetime, but individual vehicles are handled through pointers
because vehicles reference each other (leaders, followers) and a `vector<Vehicle>`
would invalidate every such pointer on any reallocation. `next_id_` is the spawn
counter used to generate each new vehicle's numeric id suffix. `arbiter_` and
`map_editor_` are constructed with member-initializer-list syntax that binds them to
*references* to `graph_`, `vehicles_`, `controller_` (and `next_id_` for the editor)
— not copies, not owned pointers.

**Why:** the comment right above this block explains the design directly: "Junction
right-of-way, lane changes and gridlock relief, and live map editing: split out of
TrafficSimulator so each has a class boundary of its own... Both are handed
references to graph_/vehicles_/controller_/next_id_ above rather than owning them —
vehicle lifetime and the graph pointer itself stay TrafficSimulator's call." This is
a deliberate ownership decision: `JunctionArbiter` needs to *read and mutate* vehicles
and the graph every tick, but it must never be the thing that decides a vehicle's
lifetime — that stays `TrafficSimulator`'s job (spawn, despawn on arrival, despawn on
removal). Giving `arbiter_` references rather than its own copies also means there's
only ever one `vehicles_` container in memory — the arbiter operates on the same
objects the simulator owns, not a stale copy.

**Usage:** because `arbiter_` and `map_editor_` are declared *after* the members they
reference, their member-initializer-list references bind correctly — C++ initializes
members in declaration order regardless of initializer-list order, so if `arbiter_`
were declared before `graph_`, it would bind to an as-yet-uninitialized `TrafficGraph*&`.
The comment even flags this explicitly: "Declared after what they reference so
constructor-order matches member-initializer-list order."

**Check yourself:**
- Why is `graph_` a raw pointer manually `delete`d in `clear()`, rather than a `std::unique_ptr<TrafficGraph>`?
- What specific bug would you get if `arbiter_` were declared *before* `graph_`/`vehicles_`/`controller_` in this class?

---

### Lines 254–277: the locking state, timers, and history/tuning fields

```cpp
std::mutex mtx_;
bool paused_ = false;
double speed_ = 1.0;
double sim_time_ = 0.0;
long tick_count_ = 0;

double sim_clock_base_epoch_ = 0.0;

bool auto_spawn_ = true;
double spawn_accum_ = 0.0;
double sample_timer_ = 0.0;

HistoryStore history_{"data/history", 86400.0};
double last_history_capture_real_ = 0.0;
int target_vehicles_ = 120;
double spawn_rate_ = 1.0;
std::string algorithm_ = "dijkstra";
CostMode cost_mode_ = CostMode::Time;
```

**Meaning:** `mtx_` is *the* simulation mutex — every public method you've read so
far that mutates or reads live state takes a `std::lock_guard<std::mutex> lock(mtx_)`
first; this is what turns "shared by the tick thread and the HTTP server thread"
from a race condition into a safely serialized sequence of operations. `sim_time_` is
simulated seconds since the last `load()`; `sim_clock_base_epoch_` is the real epoch
time `load()` last ran, used to derive a "calendar time" (`base + sim_time_`) that
advances at *simulated* speed rather than wall-clock speed — the comment on this
field explains exactly why it's not just "now": the history scrubber's clock has to
track sim speed, not real speed. `spawn_accum_` and `sample_timer_` are fractional
accumulators (explained further down where they're used). `history_` is constructed
with a 24-real-hour (`86400.0` second) rolling window. `target_vehicles_` defaults to
120, `spawn_rate_` to 1/s, `algorithm_` to `"dijkstra"`, `cost_mode_` to `CostMode::Time`
— these are the defaults a freshly booted simulator runs with before any HTTP client
changes them.

**Why:** `mtx_` being a single mutex for essentially the whole object is a coarse-grained
locking strategy — every public method blocks every other public method, including
two read-only calls that don't conflict with each other. That's a real tradeoff: simple
and provably correct (no partial-update races are possible) at the cost of some
avoidable contention, which is exactly why the *published snapshot* pattern
(`state_snapshot()`, `binary_snapshot()`) exists as an escape hatch for the hot read
path instead of trying to fix `mtx_` itself with finer-grained locking.

**Usage:** `tick()` is the single largest holder of `mtx_` per call, since all five
phases run inside one lock acquisition — see the `.cpp` walkthrough for confirmation
that the whole tick is one critical section, not five separate ones.

**Check yourself:**
- Why is there one `mtx_` for the whole simulator rather than, say, a separate mutex per subsystem (vehicles, graph, stats)? What would that buy, and what would it cost?
- `sim_clock_base_epoch_` is described as the anchor a "calendar clock" counts forward from at *simulated* speed. Why would using `std::chrono::system_clock::now()` directly, on every call, be wrong for that purpose?

---

### Lines 280–313: publisher threads, cycle-break state, and remaining members

```cpp
void publish_loop();
std::mutex snapshot_mtx_;
std::shared_ptr<const std::string> snapshot_;
std::atomic<bool> publishing_{false};

void encode_binary_snapshot_locked(std::vector<uint8_t>& buffer);
void publish_binary_snapshot_locked();
std::mutex bin_snapshot_mtx_;
std::shared_ptr<const std::vector<uint8_t>> bin_snapshot_;

void break_cycles(const std::vector<int>& waiting_on);
double cycle_timer_ = 0.0;

static double default_junction_approach(const TrafficGraph& graph);

void destroy_vehicle_locked(Vehicle* v);

std::string current_map_path_;
mutable std::mt19937 rng_{2026};
```

**Meaning:** `snapshot_mtx_` and `bin_snapshot_mtx_` are each their *own* mutex,
separate from `mtx_` — the comment is direct about why: "Reusing mtx_ would defeat
the entire point: readers would still queue behind the tick they were meant to stop
blocking." `publishing_` is an `std::atomic<bool>`, not a plain `bool`, because it's
read and written from two different threads (the caller of `start_publishing()` and
the publisher thread's own loop condition) without any mutex protecting it —
`std::atomic` is what makes that safe without a lock. `break_cycles()` and `cycle_timer_`
support the M7 gridlock-cycle search, run on its own 2-second cadence rather than
every tick (explained fully in the `.cpp` walkthrough and in the `JunctionArbiter`
section below). `default_junction_approach()` is `static` because it derives a value
purely from a `TrafficGraph` argument — it needs no `this`. `rng_` is `mutable` so it
can be advanced from `const` methods like `random_node() const`, seeded with a fixed
value (`2026`) rather than a random device — a deliberately *reproducible* seed, which
matters for a simulation whose behavior you might want to reproduce across runs while
debugging.

**Why:** the atomic flag and the two extra mutexes are both instances of the same
principle stated elsewhere in this file: the tick thread must never be made to wait on
a reader, and a reader must never be made to wait on the tick thread, for anything that
doesn't actually require shared mutable access at that instant. Fixed-seeding `rng_`
is a different but related design choice — it trades "genuinely random" for
"deterministic and debuggable," which matters more for a simulation you're actively
developing than true randomness would.

**Usage:** `publish_loop()` runs on a detached thread started once by `start_publishing()`;
it rebuilds `state_json()` every 100 ms and swaps it into `snapshot_` under `snapshot_mtx_`.
`encode_binary_snapshot_locked()`/`publish_binary_snapshot_locked()` are the binary
equivalent, but invoked from inside `tick()` itself (already holding `mtx_`) rather
than from a separate thread — hence "_locked" rather than a lock-acquiring public
wrapper for the publish step.

**Check yourself:**
- Why is `publishing_` an `std::atomic<bool>` while `paused_` (also a boolean flag, read and written across the same kind of thread boundary) is a plain `bool` guarded by `mtx_`?
- What would go wrong if `rng_` used a non-deterministic seed (e.g. `std::random_device`) while you were trying to reproduce a reported bug from a specific run?

---

## `cpp_backend/src/simulation/TrafficSimulator.cpp`

### Lines 1–41: includes and the M7 wait-for-graph constants

```cpp
#include "simulation/TrafficSimulator.hpp"
...
namespace {
    constexpr double kSpawnClearance = 3.0;

    constexpr double kCycleCheckPeriod = 2.0;
    constexpr double kWaitEdgeGap = 6.0;
    constexpr double kStoppedSpeed = 0.5;
}
```

**Meaning:** the includes bring in the full definitions this file actually needs
(`Building`, `Graph`, `Intersection`, `Road`, `TrafficLight`, `TrafficEvent`,
`MapLoader`, `Vehicle`, `VehicleTypes`) — everything the header only forward-declared.
The anonymous namespace holds tuning constants scoped to this translation unit only
(no external linkage, so they can't collide with a same-named constant in another
`.cpp`). `kSpawnClearance` is extra room a new vehicle needs at spawn beyond its own
length. The three M7 constants govern the "wait-for graph" used for gridlock cycle
detection: `kCycleCheckPeriod` (2 s between searches), `kWaitEdgeGap` (6 m — closer
than this to a stopped car counts as "queued behind it"), and `kStoppedSpeed` (0.5 m/s
below which a vehicle counts as stopped). The comment on `kWaitEdgeGap` is worth
reading closely: it says the *previous* value (12 m, matching some reference
implementation) was too generous — at 12 m the system recorded a wait-for edge for
cars simply following at a normal distance, "which handed the M7 cycle search
ordinary queueing to interpret as deadlock." That's a documented, previously-real bug:
tightening this single constant is what stopped normal traffic queues from being
misclassified as gridlock.

**Why:** this is the file's first concrete engagement with "reactive gridlock
handling" — you're looking at the exact threshold that decides whether vehicle A
counts as "waiting on" vehicle B for the purposes of deadlock *detection*. Get the gap
threshold wrong and either real deadlocks go undetected (gap too tight) or every
traffic jam gets treated as a deadlock (gap too loose, as the comment says the old
12 m value did).

**Usage:** `kWaitEdgeGap` and `kStoppedSpeed` are read in `tick_leads_and_physics()`
below, where the `waiting_on` array is built; `kCycleCheckPeriod` gates how often
`break_cycles()` runs.

**Check yourself:**
- Why would setting `kWaitEdgeGap` too large cause *false positives* in gridlock detection specifically, rather than false negatives?

---

### Lines 43–47: the Singleton definition, confirmed

```cpp
TrafficSimulator& TrafficSimulator::instance() {
    static TrafficSimulator sim;
    return sim;
}
```

**Meaning:** as previewed in the header section above, this is the full Singleton
mechanism: a function-local `static TrafficSimulator sim;`. The first call to
`instance()` from any thread constructs `sim`; every call after that, from any
thread, returns a reference to that same object. No mutex, no `std::call_once`, no
pointer, no explicit "have I been constructed yet" check anywhere in this code — the
compiler generates all of that from the `static` keyword on a function-local
variable, per the C++11 "magic statics" guarantee.

**Why:** this is Meyer's Singleton specifically, chosen (implicitly, by using this
idiom) over a heap-allocated pointer Singleton because it needs zero manual
thread-safety code to be correct under concurrent first access — exactly the
property this class needs, given it's reached from both the tick thread and the HTTP
server thread.

**Usage:** every other method in this file, and every caller anywhere else in the
codebase, reaches this object exclusively through `TrafficSimulator::instance()`.

**Check yourself:**
- If two threads call `TrafficSimulator::instance()` for the very first time at the same moment, what does the language guarantee happens, and what would you have to write by hand to get the same guarantee with a `static TrafficSimulator* sim = nullptr;` pointer instead?

---

### Lines 49–64: destructor and `clear()`

```cpp
TrafficSimulator::~TrafficSimulator() { clear(); }

void TrafficSimulator::clear() {
    for (auto& v : vehicles_) {
        controller_.detach(v.get());
    }
    vehicles_.clear();
    controller_.clear();
    controller_.set_graph(nullptr);
    delete graph_;
    graph_ = nullptr;
}
```

**Meaning:** the destructor's whole body is a call to `clear()`. `clear()` first
detaches every live vehicle from the Observer subject (`controller_.detach(v.get())`)
*before* `vehicles_.clear()` runs the `unique_ptr` destructors that actually free each
`Vehicle`. Then it clears the controller's own event state, unlinks it from the
(about to be destroyed) graph, and finally `delete graph_;` frees the raw-pointer
graph and resets it to `nullptr`.

**Why:** the comment right above this method explains the ordering requirement
precisely: "Detach before the unique_ptrs go out of scope below. The controller keeps
raw IObserver* pointers, so a vehicle freed while still subscribed leaves a dangling
entry that the next notifyAll() would call into." This is a classic Observer-pattern
lifetime hazard — see [`learn/06-events-observer.md`](06-events-observer.md) for the
full Observer mechanics — and the fix is ordering: unsubscribe before you free, never
the reverse.

**Usage:** called both when the process shuts down (destructor) and every time
`load()` installs a new map (to tear down the old world first) — `clear()` is the
single place that fully resets simulation state, so both call sites stay consistent
by construction rather than by two copies of the same teardown logic.

**Check yourself:**
- What specific crash would you expect to see (and under what conditions) if the loop order in `clear()` were reversed — `vehicles_.clear()` first, then the detach loop?

---

### Lines 66–94: `load()`

```cpp
void TrafficSimulator::load(const std::string& map_path)
{
    std::unique_ptr<TrafficGraph> loaded(new TrafficGraph());
    MapLoader::load_from_geojson(*loaded, map_path);
    std::lock_guard<std::mutex> lock(mtx_);
    clear();
    graph_ = new TrafficGraph();
    MapLoader::load_from_geojson(*graph_, map_path);
    current_map_path_ = map_path;
    controller_.set_graph(graph_);

    map_editor_.reindex();

    stats_ = StatisticsManager();
    sim_time_ = 0.0;
    sim_clock_base_epoch_ = std::chrono::duration<double>(
                                 std::chrono::system_clock::now().time_since_epoch())
                                 .count();
    tick_count_ = 0;
    next_id_ = 1;
    spawn_accum_ = 0.0;
    sample_timer_ = 0.0;
    arbiter_.reset_counters();
    arbiter_.set_junction_approach(default_junction_approach(*graph_));
}
```

**Meaning:** `load()` first parses the requested map into a *local* `unique_ptr<TrafficGraph>`
called `loaded`, entirely outside the lock and outside `graph_`. Only after that parse
succeeds (if `MapLoader::load_from_geojson` throws, this function never reaches the
lock at all) does it take `mtx_`, call `clear()` to tear down the previous world, and
then — oddly at first glance — *parse the same file again* into the real `graph_`.
Everything after that resets every piece of per-map session state to its fresh-boot
value: `sim_time_` to 0, a new `sim_clock_base_epoch_` timestamp, `tick_count_`/`next_id_`/
`spawn_accum_`/`sample_timer_` all zeroed or reset, `arbiter_.reset_counters()`, and
finally `arbiter_.set_junction_approach(default_junction_approach(*graph_))` — the
give-way distance recomputed from the *new* map's own block sizes.

**Meaning (the double-parse):** `loaded` is parsed and then discarded — it's used
purely as a validation gate; if the file is malformed, the exception propagates before
`mtx_` is ever locked and before `clear()` ever runs, so a bad map path leaves the
*currently running* city completely untouched. The real graph is then parsed a second
time, this time under the lock, directly into `graph_`.

**Why:** parsing twice looks wasteful, but it buys an important property: a failed
`load()` call is a no-op from the outside — no half-torn-down world, no held lock
during a possibly-slow file parse. The alternative (parse directly into `graph_` under
the lock) would mean an exception mid-parse leaves `graph_` in whatever partially-built
state `MapLoader` left it in, while `mtx_` is held — the worst possible failure mode
for a Singleton every other thread depends on.

**Usage:** called from `main()` at boot with the initial map path, and from `load_map()`/
`import_map()` for runtime map switches from the web UI. `default_junction_approach(*graph_)`
being called here — not once at construction — is what makes the give-way distance
"belong to the city, not to the session," per its own doc comment in the header.

**Check yourself:**
- Why does `load()` parse the map file twice instead of once, directly into `graph_`, under the lock?
- What would a failed `load()` call (bad file path) look like from the perspective of an HTTP client mid-request, given this structure?

---

### Lines 96–130: `default_junction_approach()` and the junction-approach forwarders

```cpp
double TrafficSimulator::default_junction_approach(const TrafficGraph& graph) {
    std::vector<double> lens;
    lens.reserve(graph.edges.size());
    for (const auto& [id, road] : graph.edges) {
        if (road && road->length > 0.0) lens.push_back(road->length);
    }
    if (lens.empty()) return 26.0;
    const std::size_t mid = lens.size() / 2;
    std::nth_element(lens.begin(), lens.begin() + mid, lens.end());
    const double median = lens[mid];
    return std::clamp(median * 0.5, 10.0, 26.0);
}
```

**Meaning:** collects every road's length into `lens`, then uses `std::nth_element`
to find the *median* road length in O(n) average time (a full sort would be O(n log n)
for information this function doesn't need — it only wants the middle element, not a
total order). The result is half the median block length, clamped to `[10, 26]` metres.
An empty graph falls back to the reference implementation's flat `26.0`.

**Why:** the extensive comment above this function (reproduced in the header walkthrough
above) gives real measured numbers: on `small_map`, 48% of segments are shorter than
the reference's flat 26 m constant, and using that constant put 51% of the fleet
inside the give-way window at any instant, versus 31% for the reference on the map it
was tuned for. "Half a block" is the stated rule of thumb — far enough to see the
junction coming, close enough that it's clearly about *this* junction and not the
next one down the street. `nth_element` over a full `sort` is a small but real
performance choice: this runs once per `load()`, not per tick, so it's not
performance-critical, but it's still the correct tool for "give me the median" instead
of a needlessly stronger guarantee.

**Usage:** called exactly once, from `load()`, immediately after the new graph is
built — this is what feeds `arbiter_.set_junction_approach()`. The public
`set_junction_approach()`/`junction_approach()` (below) are one-line locking forwarders
to the same-named methods on `arbiter_`.

```cpp
void TrafficSimulator::set_junction_approach(double metres) {
    std::lock_guard<std::mutex> lock(mtx_);
    arbiter_.set_junction_approach(metres);
}
double TrafficSimulator::junction_approach() const {
    return arbiter_.junction_approach();
}
```

Note `junction_approach()` is `const` and does **not** lock `mtx_` — a read of a
single `double` that JunctionArbiter itself doesn't otherwise protect, considered
cheap/safe enough to skip locking. This is a minor inconsistency worth noticing: every
other read of shared state in this class goes through the lock, but this one getter
doesn't.

**Check yourself:**
- Why `nth_element` instead of a full `std::sort` here, given this function only runs once per map load?
- `junction_approach()` reads `arbiter_`'s state without taking `mtx_`, while `set_junction_approach()` takes it before writing the same state. Is that actually safe? What would you need to know about `JunctionArbiter::junction_approach_`'s type to be sure?

---

### Lines 132–150: `tick()` — the five-phase orchestration, and where "single-threaded" actually lives

```cpp
void TrafficSimulator::tick(double dt) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (paused_ || !graph_) return;
    dt *= speed_;

    tick_events_and_lanes(dt);
    tick_leads_and_physics(dt);
    tick_cleanup_and_spawn(dt);
    tick_statistics(dt);
    tick_history_capture(dt);

    publish_binary_snapshot_locked();
}
```

**Meaning:** `tick()` takes `mtx_` once, for the entire call — every phase below runs
inside that single critical section, so from any other thread's point of view a tick
either hasn't started or has fully finished; there is no window where phase 2 has run
but phase 3 hasn't and another thread can see that half-updated state. It bails out
early (`paused_ || !graph_`) before doing any work — a paused simulation, or one with
no map loaded, does nothing. `dt *= speed_` applies the speed factor once, here, so
every phase downstream receives an already-scaled `dt` and doesn't need to know about
`speed_` at all. The five phase calls are a flat, sequential list — literally five
function calls, one after another, on the thread that called `tick()`. The trailing
`publish_binary_snapshot_locked()` call refreshes the binary snapshot before the lock
releases.

**Why — this is the single-threaded tick loop:** there is no thread pool here, no
`std::async`, no task queue splitting `vehicles_` across worker threads, no lock-free
data structure for concurrent phase execution — just five ordinary function calls
executed one after another by whichever single thread called `tick()`. Every vehicle
in `tick_leads_and_physics()`'s inner loops is visited by that same one thread, in a
plain `for` loop (you'll see this explicitly in the next section). This is the
concrete code behind the project's "single-threaded tick loop" limitation: simulating
N vehicles takes O(N) *wall-clock* time on one core, with no parallel speedup
available, no matter how many CPU cores the host machine has. The `find_package(Threads REQUIRED)`
in `CMakeLists.txt` (see [`learn/01-orientation.md`](01-orientation.md)) is not evidence
against this — it's there because the HTTP/WebSocket server needs real OS threads to
accept connections concurrently with the tick loop running, and because the publisher
thread (`publish_loop()`) and the tick thread are two *different* threads contending
for the *same* `mtx_`. Multiple threads exist in this process; only one of them is
ever inside `tick()`'s phases at a time, and no work *within* one call to `tick()` is
ever split across more than one thread.

**Usage:** `tick()` is called on a fixed cadence from a dedicated tick thread started
in `main()` (see `learn/09-server-and-entrypoint.md`) — not from an HTTP request
handler. The HTTP/WebSocket server threads never call `tick()` themselves; they call
read-only accessors (`state_snapshot()`, `binary_snapshot()`) or request-scoped
mutators (`spawn_vehicle()`, `trigger_event()`, the map-editing calls), all of which
briefly contend for the same `mtx_` the tick thread holds for the whole tick body.

**Check yourself:**
- Point to the specific evidence in `tick()` and `tick_leads_and_physics()` (below) that confirms the tick loop processes vehicles one at a time on a single thread, rather than in parallel.
- `find_package(Threads REQUIRED)` in the build file is real — so in what sense is "the tick loop is single-threaded" still an accurate description of this code, rather than a contradiction?

---

### Lines 152–191: phase 1 — `tick_events_and_lanes()`

```cpp
void TrafficSimulator::tick_events_and_lanes(double dt) {
    sim_time_ += dt;
    ++tick_count_;

    for (auto& [id, road] : graph_->edges) {
        if (road) road->update_travel_cost();
    }

    controller_.update(dt);

    int reroutes_this_tick = 0;
    constexpr int kMaxReroutesPerTick = 15;
    for (auto& v : vehicles_) {
        if (reroutes_this_tick >= kMaxReroutesPerTick) break;
        if (!v->can_recompute() || !v->get_destination()) continue;
        if (!v->consume_reroute_flag()) continue;
        const auto t0 = std::chrono::steady_clock::now();
        v->recompute_route(graph_, v->get_destination());
        const double micros = std::chrono::duration<double, std::micro>(
                                  std::chrono::steady_clock::now() - t0).count();
        stats_.recordRouteComputation(v->algorithm(), micros, v->last_nodes_explored(), !v->get_route().empty());
        reroutes_this_tick++;
    }

    arbiter_.solve_lane_changes(sim_time_);
}
```

**Meaning:** advances `sim_time_` and `tick_count_` first, so every subsequent phase
this tick reads the *new* time. Loops every road (`graph_->edges`) and calls
`update_travel_cost()` — recomputing congestion from actual live traffic, not just
from active events. `controller_.update(dt)` steps the Observer subject: events start,
tick down, and undo themselves, notifying attached vehicles (see [`learn/06-events-observer.md`](06-events-observer.md)).
Then a bounded loop over `vehicles_` reroutes at most `kMaxReroutesPerTick` (15)
vehicles that an event flagged for replanning this tick, timing each `recompute_route()`
call and feeding the microsecond timing into `stats_`. Finally `arbiter_.solve_lane_changes(sim_time_)`
runs MOBIL for the whole fleet.

**Why:** the reroute cap exists, per the inline comment, "to prevent single-tick
latency spikes when many vehicles replan" — if an event (say, a road closure) flags
hundreds of vehicles for rerouting simultaneously, recomputing all of their routes in
one tick could make that single tick take visibly longer than the others, spiking the
tick's wall-clock duration. Spreading the work across several ticks (15 per tick, the
rest wait for `consume_reroute_flag()` to still be true next tick) trades a slightly
slower *individual* reroute for a bounded, predictable per-tick cost — directly a
consequence of the tick loop being single-threaded: there's no worker pool to hand the
extra reroutes off to, so the only lever available is *deferring* work into future
ticks.

**Usage:** this is the first of the two phases the file banner says must complete
before any vehicle moves — `solve_lane_changes()` here decides who changes lanes
*before* `tick_leads_and_physics()` resolves leaders and moves everyone, so a
mid-tick lane change is visible to the leader-resolution step that follows it, not
half-applied.

**Check yourself:**
- Why is the reroute cap (`kMaxReroutesPerTick`) itself a symptom of the single-threaded tick loop, rather than an unrelated design choice?
- What would you observe in `stats_`'s route-computation timings if `kMaxReroutesPerTick` were removed entirely and a closure event flagged 500 vehicles at once?

---

### Lines 193–230: phase 2 — `tick_leads_and_physics()`, and the wait-for graph

```cpp
void TrafficSimulator::tick_leads_and_physics(double dt) {
    for (std::size_t i = 0; i < vehicles_.size(); ++i) {
        vehicles_[i]->sim_index = i;
    }

    std::vector<std::pair<Vehicle*, double>> leaders;
    leaders.reserve(vehicles_.size());
    for (auto& v : vehicles_) leaders.push_back(arbiter_.lead_in_lane(v.get()));

    std::vector<int> waiting_on(vehicles_.size(), -1);

    for (std::size_t i = 0; i < vehicles_.size(); ++i) {
        Vehicle* lead = leaders[i].first;
        if (lead && leaders[i].second < kWaitEdgeGap &&
            lead->current_speed < kStoppedSpeed) {
            waiting_on[i] = static_cast<int>(lead->sim_index);
        }
    }

    arbiter_.solve_junctions(dt, sim_time_, leaders, waiting_on);
    arbiter_.solve_gridlock(sim_time_);

    cycle_timer_ += dt;
    if (cycle_timer_ >= kCycleCheckPeriod) {
        cycle_timer_ = 0.0;
        break_cycles(waiting_on);
    }

    for (std::size_t i = 0; i < vehicles_.size(); ++i) {
        vehicles_[i]->advance(dt, leaders[i].first, leaders[i].second, sim_time_);
    }
}
```

**Meaning:** four sequential, plain `for` loops over `vehicles_`, all on the calling
thread — this is the concrete evidence for "single-threaded tick loop" promised
earlier. Loop 1 stamps each vehicle with its own index into `vehicles_` (`sim_index`)
— used later so a vehicle can be referred to by a stable integer index rather than a
pointer, which matters for the `waiting_on`/wait-for-graph arrays below, which are
plain `int` vectors indexed by vehicle position. Loop 2 builds `leaders`: for every
vehicle, ask `arbiter_.lead_in_lane()` who's ahead of it and how far — this is the
"leaders snapshotted first" the file banner promised; nobody has moved yet. Loop 3
builds `waiting_on[i]`: vehicle `i` is recorded as waiting on its leader's index if
that leader is *both* within `kWaitEdgeGap` (6 m) *and* essentially stopped
(`< kStoppedSpeed`, 0.5 m/s) — otherwise `waiting_on[i]` stays `-1` (not waiting on
anyone). Then `arbiter_.solve_junctions()` mutates `leaders` in place for vehicles that
must yield at a junction (replacing their leader with the stop line — see the
`JunctionArbiter` section), and can also add wait-for edges to `waiting_on` when a
junction blocker is genuinely parked. `arbiter_.solve_gridlock()` runs the M7
last-resort rescue. `cycle_timer_` accumulates `dt` and only every `kCycleCheckPeriod`
(2 s) triggers `break_cycles(waiting_on)` — deliberately not every tick, because "a
deadlock that exists now still exists in two seconds, and the search is O(n) each
time," per the inline comment. Loop 4, the last one, is where motion actually happens:
`vehicles_[i]->advance(dt, leaders[i].first, leaders[i].second, sim_time_)` — every
vehicle moves using the *frozen* `leaders` snapshot from loop 2 (possibly modified by
`solve_junctions()`), not a freshly re-resolved leader.

**Why (single-threaded, concretely):** every one of these four loops is a plain,
sequential `for` over the same `std::vector<std::unique_ptr<Vehicle>>&`, executed by
whichever one thread is inside `tick()`. There's no `std::for_each(std::execution::par, ...)`,
no manual thread partitioning of the vehicle array, no atomics protecting concurrent
writes to `waiting_on`/`leaders` — because nothing ever writes to them from more than
one thread. This is also *why* it's structured as "resolve everyone, then move
everyone" rather than "resolve-and-move" fused into one loop: it's not a parallelism
optimization (there is no parallelism), it's what guarantees a vehicle's decision
doesn't depend on iteration order within the single thread that is doing the work.

**Why (the wait-for graph is reactive, not preventive):** `waiting_on` is built by
*observing* which vehicles are already stopped near each other — it's evidence
collected *after* the fact of near-gridlock conditions existing, not a check performed
*before* allowing a vehicle to enter a state that could deadlock. `break_cycles()`
(only every 2 real seconds, not continuously) then walks this graph looking for
*cycles* — closed rings of "waiting on" relationships that can never resolve on their
own. Nothing here stops a gridlock-prone situation from forming in the first place;
the detection happens strictly after several vehicles are already stopped and
blocking each other. This is the concrete meaning of "gridlocks are handled reactively
rather than prevented": there is no admission control anywhere in `solve_junctions()`
or elsewhere that refuses to let a vehicle enter a junction box if doing so would risk
a future deadlock — the system lets it happen, waits up to 2 seconds, then goes
looking for cycles in the wreckage and forces one vehicle per cycle through.

**Usage:** `waiting_on` built here is passed both into `arbiter_.solve_junctions()`
(which can add to it) and into `break_cycles()` — a single array threading through
detection (built here), refinement (junction yielding), and resolution
(`break_cycles`/`solve_gridlock`).

**Check yourself:**
- `waiting_on[i]` records the index of the *single* vehicle `i` is waiting on. What property of this graph (stated explicitly in `JunctionArbiter.hpp`'s comments) does that "at most one" constraint make possible for `break_cycles()`'s implementation?
- Why does `break_cycles()` run only every `kCycleCheckPeriod` (2 s) rather than every tick, and what specific comment in the code justifies that choice?
- Walk through what would have to change in this function for gridlock to be *prevented* rather than *detected-and-broken* — what check would need to run, and where, before a vehicle is allowed to advance into a junction box?

---

### Lines 232–266: phases 3 and 4 — cleanup/spawn and statistics

```cpp
void TrafficSimulator::tick_cleanup_and_spawn(double dt) {
    for (auto it = vehicles_.begin(); it != vehicles_.end();) {
        Vehicle* v = it->get();
        if (v->has_reached_destination()) {
            double length = 0.0;
            for (Road* r : v->get_route()) length += r->length;
            stats_.recordTripCompleted(v->travel_time(), length, v->type());
            controller_.detach(v);
            it = vehicles_.erase(it);
        } else {
            ++it;
        }
    }
    if (auto_spawn_) auto_spawn(dt);
}
```

**Meaning:** a manual iterator loop (not range-`for`, because it needs to erase
elements mid-iteration) over `vehicles_`, removing any vehicle that has reached its
destination — recording its completed-trip stats, detaching it from the Observer
subject (same lifetime hazard as `clear()`: detach before the `unique_ptr` frees the
vehicle on `erase()`), and advancing the iterator via `erase()`'s return value rather
than a separate `++it`. Vehicles that haven't arrived just get `++it`. Once cleanup is
done, `auto_spawn(dt)` runs if `auto_spawn_` is enabled.

```cpp
void TrafficSimulator::tick_statistics(double dt) {
    sample_timer_ += dt;
    if (sample_timer_ < 2.0) return;
    sample_timer_ = 0.0;
    double total = 0.0;
    for (const auto& [id, road] : graph_->edges) {
        if (road) total += road->get_congestion_level();
    }
    const double avg = graph_->edges.empty() ? 0.0 : total / static_cast<double>(graph_->edges.size());
    stats_.pushSample(sim_time_, avg, static_cast<int>(vehicles_.size()));
}
```

**Meaning:** `sample_timer_` accumulates real-`dt` and only every 2 (simulated)
seconds does the actual work — average congestion across every road, pushed to
`stats_` for the chart along with the current time and fleet size.

**Why:** the comment on `tick_statistics` explains the throttle directly: "60 samples
a second would be noise and would swamp the history buffer in under a minute" — 20 Hz
ticks would produce 20 samples/second of essentially the same congestion number if
sampled every tick; sampling every 2 s produces a usable chart resolution without
bloating `stats_`'s sample buffer.

**Usage:** `tick_cleanup_and_spawn()` is the only place vehicles are actually removed
from `vehicles_` mid-run (as opposed to `remove_vehicle()`/`clear_vehicles()`, which
are explicit API calls) — it's what keeps the fleet from growing forever as trips
complete. `tick_statistics()` feeds the congestion-over-time chart in the frontend.

**Check yourself:**
- Why does `tick_cleanup_and_spawn()` use a manual iterator loop with `it = vehicles_.erase(it)` instead of, say, `std::erase_if(vehicles_, ...)`? (Hint: what side effect — `controller_.detach`, `stats_.recordTripCompleted` — has to happen for exactly the vehicles being erased, and in what order relative to the erase?)
- What would the congestion chart look like if `tick_statistics()`'s 2-second throttle were removed and it sampled every tick instead?

---

### Lines 268–353: phase 5 — `tick_history_capture()`

```cpp
void TrafficSimulator::tick_history_capture(double /*dt*/) {
    const double now = std::chrono::duration<double>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count();
    if (now - last_history_capture_real_ < 1.0) return;
    last_history_capture_real_ = now;

    HistoryFrame f;
    f.real_time = now;
    f.sim_time = sim_time_;
    f.calendar_time = sim_clock_base_epoch_ + sim_time_;
    ...
    history_.capture(f);
}
```

**Meaning:** the parameter `dt` is explicitly unused (`/*dt*/`) — this phase is gated
by *real* elapsed time (`std::chrono::system_clock::now()`), not by `dt`, which is
already scaled by `speed_` by the time it reaches here. It captures at most once per
real second. The body builds a `HistoryFrame` — vehicles, signalized lights, congested/closed
roads, active events, and aggregate stats — by walking the same live objects
`state_json()` walks, but into `HistoryFrame`'s shape instead of a JSON string, then
hands it to `history_.capture(f)`.

**Why:** the comment is explicit that gating on real time (not `dt`) is deliberate:
"capturing on simulated time would make total data volume for the same real 24h
window vary by up to 20x depending on what speed the map happened to run at, defeating
the point of a size-bounded window." If capture were gated on `sim_time_ - last >= 1.0`
instead, running the simulation at 10x speed would capture 10x as many frames per real
hour, blowing well past whatever storage budget a fixed real-time window was sized
for. Real-time gating keeps the data volume for a fixed real-world window constant
regardless of the speed slider. The comment also explicitly flags that this walk and
`state_json()`'s walk are *not* shared code — a deliberate duplication, to avoid
risking `state_json()`'s many existing callers/tests by refactoring it to share logic
with history capture — with the tradeoff spelled out: if a field's meaning changes on
one side, the other has to be checked and updated by hand.

**Usage:** `history_.capture(f)` is what the playback scrubber (`history_range_json()`/
`history_state_at_json()`) eventually reads back from.

**Check yourself:**
- Why would gating this capture on `sim_time_` (already scaled by `speed_`) rather than real wall-clock time defeat the purpose of a fixed 24-hour history window?
- What's the concrete risk called out in the comment about state_json() and tick_history_capture() being two separate, unshared walks over the same objects?

---

### Lines 356–434: `spawn_vehicle()` / `spawn_locked()` — the Factory, and lane selection

```cpp
std::string TrafficSimulator::spawn_vehicle(const std::string& from, const std::string& to,
                                            const std::string& type) {
    std::lock_guard<std::mutex> lock(mtx_);
    return spawn_locked(from, to, type);
}
```

**Meaning:** confirms the pattern flagged in the header — the public method locks,
then delegates to the `_locked` variant that assumes the lock is already held.

The body of `spawn_locked()` validates `from`/`to` exist and differ, builds the new
vehicle's id by combining `VehicleFactory::canonical_type(type)` with `next_id_`
(explained in a comment: the id's type prefix must exactly match one of the frontend's
`TYPE_NAMES` entries, because the binary snapshot protocol reconstructs a display id
from `TYPE_NAMES[type] + num_id` rather than sending the string id at all), constructs
the vehicle via `VehicleFactory::create()` (see [`learn/05-vehicles.md`](05-vehicles.md)),
requests a route, and bails out with `""` if the route came back empty (unreachable
destination) — `owned` (the local `unique_ptr`) simply goes out of scope and frees the
vehicle on that path, no manual `delete` needed.

**Meaning (lane selection):**

```cpp
if (Road* first = v->current_road) {
    const std::vector<int> ok = v->allowed_lanes(first->get_lanes());
    const double needed = v->length_meters() + kSpawnClearance;
    std::vector<int> best_lanes;
    std::size_t fewest = std::numeric_limits<std::size_t>::max();
    for (int l : ok) {
        bool slot_ok = true;
        std::size_t occupants = 0;
        for (Vehicle* o : first->vehicles) {
            if (o == v || o->lane != l) continue;
            ++occupants;
            if (o->position_on_road < needed) { slot_ok = false; break; }
        }
        if (!slot_ok) continue;
        if (occupants < fewest) { fewest = occupants; best_lanes.clear(); }
        if (occupants == fewest) best_lanes.push_back(l);
    }
    if (best_lanes.empty()) { first->remove_vehicle(v); return ""; }
    std::uniform_int_distribution<std::size_t> pick(0, best_lanes.size() - 1);
    v->lane = best_lanes[pick(rng_)];
}
```

For each allowed lane on the first road, this counts occupants and rejects the lane
outright if anything sits inside the `needed` clearance from the start. Among the
remaining lanes, it keeps only the ones tied for *fewest* occupants, then picks
uniformly at random among that tied set. If every allowed lane is blocked at the
entrance, the vehicle is removed from the road and spawn fails (`""`).

**Why:** the long comment above this block documents a *fixed* bug: the previous rule
picked the lane with the largest "clearest gap ahead," initialized to `1e9` and
compared with strict `>`. On an empty or lightly used road, every lane ties at exactly
`1e9`, so a strict `>` comparison never replaces the first candidate — "every vehicle
in the city spawned in lane 0, and because MOBIL only moves a vehicle that is actually
held up, it stayed there. A 3-lane road ran as a 1-lane road." The fix — track fewest
occupants, break ties randomly rather than by lane index — spreads spawns across lanes
even when there's no congestion yet to force a MOBIL lane change later.

**Usage:** `spawn_locked()` is called from `spawn_vehicle()` (public, locking) and
from `auto_spawn()` (already inside `tick()`'s lock).

**Check yourself:**
- Why did the old "largest clearest gap, strict `>` comparison" rule specifically fail on an *empty* road rather than a busy one?
- Why does the lane-tie-break pick randomly among tied lanes instead of, say, always picking the lowest-numbered tied lane?

---

### Lines 436–461, 508–587: events, playback setters, and the two publisher loops

```cpp
std::string TrafficSimulator::trigger_event(const std::string& type, const std::string& target,
                                            double duration) {
    std::lock_guard<std::mutex> lock(mtx_);
    return controller_.trigger_event(type, target, duration);
}
```

**Meaning:** locks, then forwards straight to `controller_` — `TrafficController`
owns events entirely; `TrafficSimulator` doesn't touch event internals at all. The
playback setters (`set_paused`, `set_speed`, `set_auto_spawn`, `set_random_events`)
that follow are all the same shape: lock, assign one field (or forward one call),
unlock. `set_speed` clamps its input to `[0.1, 10.0]` — the simulation can never be
fully stopped through this knob (use `set_paused` for that) nor sped up past 10x.

```cpp
void TrafficSimulator::start_publishing() {
    if (publishing_.exchange(true)) return;
    history_.reset();
    std::thread(&TrafficSimulator::publish_loop, this).detach();
}

void TrafficSimulator::publish_loop() {
    while (publishing_.load()) {
        auto fresh = std::make_shared<const std::string>(state_json());
        {
            std::lock_guard<std::mutex> lock(snapshot_mtx_);
            snapshot_ = std::move(fresh);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
```

**Meaning:** `publishing_.exchange(true)` atomically sets the flag to `true` and
returns its *previous* value in one operation — if it was already `true`, this call
returns immediately (idempotent: calling `start_publishing()` twice starts only one
thread). `history_.reset()` wipes any on-disk playback history from a prior process
run — the comment explains this is done here specifically because `start_publishing()`
is already the guarded "run exactly once at boot" entry point, so reusing it avoids a
second explicitly-ordered boot step. The thread is `.detach()`ed, not joined — per the
comment, "the simulator is a singleton that lives as long as the process, so there is
no shutdown to join at." `publish_loop()` itself is a `while (publishing_.load())` loop
that rebuilds `state_json()` (which internally takes and releases `mtx_` for the
duration of that one call, not for the sleep), swaps it into `snapshot_` under
`snapshot_mtx_` — its *own* mutex, distinct from `mtx_` — then sleeps 100 ms.

**Why:** this is the second thread the class banner promised exists alongside the tick
thread — `publish_loop()` runs on its own detached thread, calling `state_json()`
(which locks `mtx_` briefly) once per 100 ms cycle, entirely independent of when
`tick()` happens to run. `state_json()` and `tick()` still serialize against each
other through `mtx_` — this doesn't parallelize the *simulation* itself, only the
*publishing of already-computed state* alongside it.

**Usage:** `start_publishing()` is called once from `main()`; `state_snapshot()`
(public, in the header) reads `snapshot_` under `snapshot_mtx_` with no lock on `mtx_`
at all — the actual payoff of running this on a separate thread.

**Check yourself:**
- `publishing_.exchange(true)` versus separately checking `if (!publishing_.load()) { publishing_.store(true); ... }` — why is the single `exchange` call necessary for correctness under concurrent calls to `start_publishing()`?
- The publish thread is detached rather than stored and joined at shutdown. Given this is a Singleton that "lives as long as the process," what would joining it at shutdown actually require, and why does the code sidestep that requirement instead?

---

### Lines 595–719: `state_json()`, and Lines 742–790: the binary snapshot pair

`state_json()` is a long, single method that builds one JSON object by hand with
`std::ostringstream`, field by field: `time`, `calendarTime` (explicitly cast to
`long long` before insertion — the comment explains `ostringstream`'s default 6
significant digits for a `double` would throw away everything below ~10000 s
resolution for a ~1.7-billion-second epoch value, misreporting the clock by up to
~3 hours), `paused`, `speed`, the control-panel echo fields (`targetVehicles`,
`spawnRate`, `junctionApproach`, `algorithm`, `autoSpawn`, `randomEvents`), then arrays
for `vehicles`, `lights` (one entry **per approach**, not per junction — the comment
explains a junction shows different colours to different approaches, so a
junction-keyed entry couldn't describe that), `roads` (congested or closed only, to
keep the payload small), `events`, a rolling `log`, the `solver` counters read
straight off `arbiter_` (`lane_changes()`, `yields()`, `box_waits()`, `forced_throughs()`,
`rescues()`), and finally `active`/`spawned`/`stats`.

**Why the solver counters matter here:** `state_json()`'s `"solver"` object is the
UI's window into the arbiter's behavior — `yields`/`box_waits` distinguish two
different reasons a vehicle stopped (per `JunctionArbiter.hpp`'s own comment: "yields_
is 'I gave way to another stream'; box_waits_ is 'the road beyond the junction is
full, so I stopped at the line rather than stall across it'... A rise in the first
says right of way is contested; a rise in the second says the network is backing up"),
and `forced_throughs`/`rescues` are the two-stage reactive-gridlock counters — every
increment of either is a moment the M7 machinery had to intervene because normal
right-of-way rules alone left a vehicle permanently stuck. A dashboard showing these
rising over a run is a direct, visible readout of how often the "reactive, not
preventive" gridlock handling actually had to fire.

`encode_binary_snapshot_locked()`/`publish_binary_snapshot_locked()`/`binary_snapshot()`
are the binary-format equivalents already previewed in the header — filling
`VehicleSnapshotBin` structs directly via `memcpy` into a caller-or-shared buffer, one
struct per live vehicle, with a small `binary_type_code()` helper mapping the
vehicle's string type to the fixed numeric code the frontend's `TYPE_NAMES` table
expects.

**Check yourself:**
- If you saw `forced_throughs` and `rescues` both climbing steadily over a long-running simulation (not just spiking briefly during a stress test), what would that tell you about the health of the underlying traffic model, per the comment on `solve_gridlock()`?
- Why is `lights` one JSON entry per road-approach rather than one per junction node?

---

### Lines 1008–1063: `import_map()` and the `break_cycles()` forwarder

`import_map()` was already covered in the header, but the `.cpp` body is worth
confirming: it parses into a throwaway `candidate` graph first purely as a validation
gate (`candidate.reset()` immediately after — "load() below re-parses; this was only
the gate"), collects a non-fatal `warning` from `MapLoader::validate()`, writes the raw
GeoJSON to disk only once parsing succeeded, and only then calls `load(path)` — the
same double-parse-for-safety pattern `load()` itself uses, one layer up.

```cpp
void TrafficSimulator::break_cycles(const std::vector<int>& waiting_on) {
    arbiter_.break_cycles(waiting_on, sim_time_);
}
```

**Meaning:** the private `break_cycles()` you saw declared in the header, and called
from `tick_leads_and_physics()` on its 2-second cadence, is a one-line forwarder to
`arbiter_.break_cycles()` — `TrafficSimulator` doesn't implement cycle detection
itself at all; it only owns the *timer* that decides when to ask `JunctionArbiter` to
run it.

**Why:** this is the last piece of the ownership split promised at the top of this
module — `TrafficSimulator` orchestrates *when* things happen (the tick, the cadence),
`JunctionArbiter` decides *what* happens (whose leader gets replaced, which cycle gets
broken).

**Check yourself:**
- Why does `TrafficSimulator::break_cycles()` exist as a private forwarder instead of `tick_leads_and_physics()` just calling `arbiter_.break_cycles(waiting_on, sim_time_)` directly?

---

**Check yourself (whole file):**
- Trace one full `tick()` call end to end: which phase decides a vehicle should give way at a junction, which phase actually moves it, and which data structure carries that decision from one phase to the other?
- Name every `std::mutex` this file declares (`mtx_`, `snapshot_mtx_`, `bin_snapshot_mtx_`) and, for each, state in one sentence what it protects and why it isn't just `mtx_`.

---

## `cpp_backend/include/simulation/JunctionArbiter.hpp`

### Lines 1–18: the file banner — what's being arbitrated, and why it's not `Vehicle`'s job

```cpp
#pragma once
/*
JunctionArbiter.hpp

- Owns the per-tick traffic solver: MOBIL lane changes, right-of-way at
  unsignalized junctions, and gridlock detection/relief. A Vehicle can only
  see its own road, so everything here needs a global view across the fleet
  -- that is what separates this from Vehicle::advance(), which only needs
  what the arbiter hands it (a leader and a gap).

- Split out of TrafficSimulator so the solver -- the most complex, most
  frequently retuned part of the simulation -- has a class boundary of its
  own, matching the assignment's suggested Simulation-component split.
  TrafficSimulator still drives the tick and owns the vehicles/graph; this
  class is handed references to both rather than owning either, since a
  vehicle's fate (spawn/despawn) is the simulator's call, not the
  arbiter's.
*/
```

**Meaning:** this states outright what "arbiter" means in this codebase: it is the
one place with a view across the whole fleet, used specifically because an individual
`Vehicle` (see [`learn/05-vehicles.md`](05-vehicles.md)) cannot see beyond its own
road — `Vehicle::advance()` only ever receives a leader and a gap distance, computed
for it by this class. Three responsibilities are named explicitly: MOBIL lane changes,
unsignalized-junction right of way, and gridlock detection/relief — the exact three
things this module needs to explain in depth.

**Why:** the comment gives the actual reason for splitting this out of `TrafficSimulator`:
it's "the most complex, most frequently retuned part of the simulation" — code that
changes often benefits from its own compilation unit and its own class boundary,
separate from the orchestration logic in `TrafficSimulator` that changes rarely by
comparison. The ownership note repeats what you already saw in `TrafficSimulator.hpp`:
`JunctionArbiter` is hooked up via references, not ownership, because vehicle
lifetime is explicitly *not* this class's call.

**Usage:** every method on this class is called from inside `TrafficSimulator`'s tick
phases — `solve_lane_changes()` from phase 1, `lead_in_lane()`/`solve_junctions()`/
`solve_gridlock()`/`break_cycles()` from phase 2, as you already saw in the `.cpp`
walkthrough above.

**Check yourself:**
- Concretely, what does `Vehicle::advance()` know about the rest of the fleet, and what does it rely entirely on `JunctionArbiter` to have already decided for it?

---

### Lines 20–42: includes, forward declarations, and `find_gridlock_cycle_leaders()`

```cpp
#include <cstddef>
#include <memory>
#include <utility>
#include <vector>

class TrafficGraph;
class TrafficController;
class Vehicle;
class Road;

std::vector<std::size_t> find_gridlock_cycle_leaders
    (const std::vector<int>& waiting_on, const std::vector<double>& stuck_times);
```

**Meaning:** four forward-declared classes this header only ever references through
pointers or references — same include-minimization reasoning as `TrafficSimulator.hpp`.
`find_gridlock_cycle_leaders()` is declared as a **free function**, outside the
`JunctionArbiter` class entirely, taking two plain arrays (`waiting_on`, `stuck_times`)
and returning a list of vehicle indices.

**Why:** the comment is explicit about why this is a free function rather than a
private method: "it is the whole decision, it depends on nothing but two arrays, and
that makes it the part worth testing directly." This is a real, deliberate testability
choice — a pure function over two `vector`s can be unit-tested with hand-constructed
input arrays and no `JunctionArbiter`, no `TrafficGraph`, no `Vehicle` fleet required
at all. `tests/unit_tests/` almost certainly exercises this function directly (see the
`test_gridlock_stress` executable named in `learn/01-orientation.md`'s coverage of
`CMakeLists.txt`).

**Usage:** called from `JunctionArbiter::break_cycles()` further down — this free
function *is* the cycle-detection algorithm; `break_cycles()` is just the glue that
prepares its inputs and acts on its output.

**Check yourself:**
- What would you lose, in terms of testability, if `find_gridlock_cycle_leaders()` were a private member function of `JunctionArbiter` instead of a free function?

---

### Lines 44–52: the class, constructor, and `lead_in_lane()`

```cpp
class JunctionArbiter {
public:
    JunctionArbiter(TrafficGraph*& graph, std::vector<std::unique_ptr<Vehicle>>& vehicles,
                    TrafficController& controller);

    std::pair<Vehicle*, double> lead_in_lane(Vehicle* v) const;
```

**Meaning:** the constructor takes `TrafficGraph*&` — a *reference to a pointer*, not
a plain pointer — plus a reference to the vehicle vector and a reference to the
controller. `lead_in_lane()` is `const` (it doesn't mutate arbiter state) and returns
`std::pair<Vehicle*, double>` — the nearest leader and the gap to it, or `{nullptr, ...}`
if nothing's ahead.

**Why:** `TrafficGraph*&` specifically — a reference to a pointer, not just a pointer
— is what lets `JunctionArbiter` always see the *current* `graph_` even after
`TrafficSimulator::load()` reassigns it to a brand-new `TrafficGraph*`. If the
constructor took `TrafficGraph*` by value, `JunctionArbiter` would capture whatever
`graph_` pointed to *at construction time* and never notice a later `load()` swapping
it out — it would keep dereferencing a freed graph after the very first map reload.
Binding to the *reference* (`graph_` the member, `graph_&` here) means every read
through this reference always sees `TrafficSimulator::graph_`'s current value.

**Usage:** `lead_in_lane()` is called both from `TrafficSimulator::tick_leads_and_physics()`
(building the tick's `leaders` snapshot) and internally from `accel_in_lane()` for
MOBIL's projected-acceleration calculations.

**Check yourself:**
- Why does the constructor take `TrafficGraph*&` (reference to pointer) instead of `TrafficGraph*` (a plain pointer copy)? What specific bug would a plain-pointer parameter introduce the moment a map is reloaded?

---

### Lines 54–86: the four solver methods and the give-way knob

```cpp
void solve_lane_changes(double sim_time);

void solve_junctions(double dt, double sim_time,
                     std::vector<std::pair<Vehicle*, double>>& leaders,
                     std::vector<int>& waiting_on);

void break_cycles(const std::vector<int>& waiting_on, double sim_time);

void solve_gridlock(double sim_time);

void set_junction_approach(double metres);
double junction_approach() const { return junction_approach_; }
```

**Meaning:** four methods, each with an "M-number" in its doc comment (`M3/M4`,
`M6`, `M7 stage 1`, `M7 stage 2`) — these are milestone labels from the project's own
development history, and they establish an order of increasing sophistication:
MOBIL lane changes and right-of-way came first, then two *stages* of gridlock
handling layered on top once plain right-of-way alone turned out to leave vehicles
permanently stuck. Note `solve_junctions()` takes `leaders` and `waiting_on` by
non-const reference — it *mutates both*, which is why `tick_leads_and_physics()`
builds them first and passes them in rather than this method returning a value.
`set_junction_approach()`/`junction_approach()` are the same give-way distance knob
you already saw forwarded from `TrafficSimulator`, clamped here to `[3, 60]` (note:
tighter than the `[7, 60]` the `TrafficSimulator.hpp` comment quotes for the *public*
API — the arbiter's own clamp is a wider safety net; `TrafficSimulator`'s narrower
`[7, 60]` in its own doc comment is presumably the intended UI-facing range).

**Why:** the two-stage naming (`M7 stage 1` = `break_cycles`, `M7 stage 2` =
`solve_gridlock`) tells you the gridlock-relief mechanism is itself layered: stage 1
tries to find and break an actual deadlock *cycle* using graph structure; stage 2 is a
blunter fallback for a vehicle that's stuck but not part of any cycle the wait-for
graph can name (its comment, covered below, calls this "the escape hatch of last
resort").

**Usage:** all four are called from `TrafficSimulator`'s phase 1/2 tick methods, in
the sequence already traced in the `.cpp` walkthrough: `solve_lane_changes()` in phase
1; `solve_junctions()`, `solve_gridlock()`, and (on its own throttled cadence)
`break_cycles()` in phase 2.

**Check yourself:**
- Why does `solve_junctions()` take `leaders` and `waiting_on` as non-`const` references rather than returning a new pair of vectors?
- What does the "M7 stage 1 / stage 2" naming tell you about the order these two gridlock mechanisms were added, and why might a project add a blunter "stage 2" *after* a more principled "stage 1" already existed?

---

### Lines 79–118: solver counters and private state

```cpp
long lane_changes() const { return lane_changes_; }
long yields() const { return yields_; }
long box_waits() const { return box_waits_; }
long forced_throughs() const { return forced_throughs_; }
long rescues() const { return rescues_; }
void reset_counters();

private:
    double accel_in_lane(Vehicle* v, int lane) const;
    Vehicle* follower_in_lane(Vehicle* v, int lane) const;

    TrafficGraph*& graph_;
    std::vector<std::unique_ptr<Vehicle>>& vehicles_;
    TrafficController& controller_;

    double junction_approach_ = 26.0;

    struct JunctionOccupant {
        std::size_t vehicle_idx;
        const Road* road;
    };
    std::vector<std::vector<JunctionOccupant>> node_occupants_;
    std::vector<std::vector<std::size_t>> node_approaching_;

    long lane_changes_ = 0;
    long yields_ = 0;
    long box_waits_ = 0;
    long forced_throughs_ = 0;
    long rescues_ = 0;
};
```

**Meaning:** five public getters exposing five private counters — the same counters
`state_json()` reads for the `"solver"` JSON object. The doc comment distinguishes
`yields_` from `box_waits_` explicitly (already quoted above): contested right of way
versus network backing up. `accel_in_lane()`/`follower_in_lane()` are private MOBIL
helpers. `graph_`, `vehicles_`, `controller_` are the three references from the
constructor, stored as references (not pointers or copies) — matching the
`TrafficGraph*&` reasoning above. `node_occupants_`/`node_approaching_` are per-junction
scratch state, indexed by `Intersection::index`, resized lazily — who's physically in
each junction's box this tick (`node_occupants_`) and who's approaching it
(`node_approaching_`); both rebuilt fresh every call to `solve_junctions()` (you'll see
the `.clear()` calls in the `.cpp`).

**Why:** `node_occupants_`/`node_approaching_` being members (not local variables
inside `solve_junctions()`) rather than freshly-allocated locals every call is a
performance choice — reusing the same outer `vector`s across ticks means their inner
`vector`s' capacity is retained, avoiding repeated heap allocation for what is, in a
running simulation, roughly the same number of vehicles near roughly the same number
of junctions every tick.

**Usage:** `reset_counters()` is called once from `TrafficSimulator::load()` — a fresh
map starts with fresh solver statistics rather than carrying counts forward from a
previous city.

**Check yourself:**
- Why are `node_occupants_`/`node_approaching_` declared as member variables that get cleared and refilled every call, rather than local variables constructed fresh inside `solve_junctions()` each time?

---

## `cpp_backend/src/simulation/JunctionArbiter.cpp`

### Lines 1–61: includes and the tuning constants — where the numbers behind every rule live

```cpp
namespace {
    constexpr double kMobilSafeDecel = -2.0;
    constexpr double kMobilThreshold = 0.35;
    constexpr double kMobilReturnThreshold = 1.0;
    constexpr double kMobilObstruction = 0.4;
    constexpr double kLaneChangeMinGap = 1.0;

    constexpr double kJunctionZone = 2.0;
    constexpr double kZoneApproachMax = 26.0;
    constexpr double kZoneHorizon = 4.0;
    constexpr double kZoneTieMargin = 0.3;
    constexpr double kZoneBonusCap = 3.0;
    constexpr double kMovingSpeed = 0.5;
    constexpr double kZoneMinSpeed = 1.5;
    constexpr double kIgnoreBlockerAfter = 15.0;
    constexpr double kStoppedSpeed = 0.5;

    constexpr double kForcedGrant = 8.0;
    constexpr double kStuckRescue = 45.0;

    constexpr double kCrossRoadLookahead = 60.0;

    double gap_between(const Vehicle* back, const Vehicle* front) {
        return front->position_on_road - back->position_on_road -
               0.5 * (front->length_meters() + back->length_meters());
    }
}
```

**Meaning:** every numeric rule this class enforces is named and commented here, up
front, grouped by which mechanism it belongs to (MOBIL, junction yielding M6,
gridlock breaking M7, cross-road lookahead). `kLaneChangeMinGap`'s comment (quoted in
full further below) is the single most important one for the junction-overlap
question this module has to answer honestly. `gap_between()` is a small free function:
bumper-to-bumper distance between two vehicles on the same road, subtracting half of
each vehicle's own length from the raw position difference — this is used throughout
the file as *the* definition of "gap" between two vehicles.

**Why:** collecting every tunable threshold into one commented block at the top of the
file (rather than scattering magic numbers through the logic below) is what makes this
"the most complex, most frequently retuned part of the simulation" (per the header
banner) actually retunable — someone adjusting how aggressively vehicles change lanes
only has to touch `kMobilThreshold`, not hunt through 500 lines of control flow for
every place `0.35` might appear.

**Usage:** every constant here is read at least once in the methods below; you'll see
each one in context.

**Check yourself:**
- Why does `gap_between()` subtract *half* of each vehicle's length rather than the full length of one or the other?

---

### Lines 77–120: `lead_in_lane()` — looking past the end of the road

```cpp
std::pair<Vehicle*, double> JunctionArbiter::lead_in_lane(Vehicle* v) const {
    Vehicle* best = nullptr;
    double best_gap = 1e9;
    if (!v->current_road) return {nullptr, 1e9};
    for (Vehicle* o : v->current_road->vehicles) {
        if (o == v || o->lane != v->lane) continue;
        if (o->position_on_road <= v->position_on_road) continue;
        const double g = gap_between(v, o);
        if (g < best_gap) { best_gap = g; best = o; }
    }
    if (best) return {best, std::max(0.0, best_gap)};

    const double to_end = v->current_road->length - v->position_on_road;
    if (to_end > kCrossRoadLookahead) return {nullptr, 1e9};

    const std::vector<Road*>& route = v->get_route();
    const int next_index = v->get_route_index() + 1;
    if (next_index <= 0 || next_index >= static_cast<int>(route.size()))
        return {nullptr, 1e9};
    Road* next = route[next_index];
    if (!next) return {nullptr, 1e9};

    int lane = v->lane;
    const std::vector<int> allowed = v->allowed_lanes(next->get_lanes());
    if (std::find(allowed.begin(), allowed.end(), lane) == allowed.end())
        lane = allowed.empty() ? 0 : allowed.front();

    for (Vehicle* o : next->vehicles) {
        if (o == v || o->lane != lane) continue;
        const double g = to_end + o->position_on_road - o->length_meters();
        if (g < best_gap) { best_gap = g; best = o; }
    }
    return {best, std::max(0.0, best_gap)};
}
```

**Meaning:** first scans every vehicle `o` on the *same road and lane* as `v` that is
ahead of it (`o->position_on_road > v->position_on_road`), keeping whichever has the
smallest `gap_between()`. If nothing's found on the current road, and the remaining
distance to the road's end is within `kCrossRoadLookahead` (60 m), it looks *into the
next road on the route* — remapping the lane through `allowed_lanes()` since a road's
lane numbering isn't guaranteed to match the next road's — and computes a gap that
*spans the junction*: distance left on the current road, plus how far the leader has
already travelled into the next road, minus the leader's own length.

**Why:** the comment explains exactly what this second half fixes: "Without this a
vehicle never sees a queue that has spilled back through the junction: Vehicle::advance()
hands it over at position ~0 and drops it on top of whoever is already waiting there."
Without cross-road lookahead, a vehicle approaching a junction with a backed-up queue
just beyond it would have no leader at all (nothing ahead on its *own* road) and
accelerate right up to the road's end, then get handed to the next road at position 0
— directly on top of a vehicle already stopped there. `kCrossRoadLookahead` (60 m) is
capped because braking from the fastest desired speed (~22 m/s) at the IDM's comfort
deceleration takes about 81 m in theory, but "the junction solver already slows
traffic on the approach, so 60 m is enough in practice and keeps the scan cheap" — an
explicit performance/accuracy tradeoff, not a physical necessity.

**Usage:** the primary consumer of `lead_in_lane()` is `TrafficSimulator::tick_leads_and_physics()`,
building the tick's `leaders` snapshot for every vehicle before anyone moves; it's also
used inside `accel_in_lane()` for MOBIL's own-lane projection below (indirectly — `accel_in_lane`
reimplements the same-road half of this search rather than calling `lead_in_lane()`,
since it needs to search an *arbitrary* candidate lane, not just `v`'s current one).

**Check yourself:**
- What specific bug — described in the code's own comment — does the cross-road lookahead half of `lead_in_lane()` fix, and what would you observe on screen if it were removed?
- Why is `kCrossRoadLookahead` set to 60 m rather than the ~81 m the full IDM braking-distance calculation would suggest?

---

### Lines 122–152: `accel_in_lane()` and `follower_in_lane()` — MOBIL's two probes

```cpp
double JunctionArbiter::accel_in_lane(Vehicle* v, int lane) const {
    Road* road = v->current_road;
    if (!road) return 0.0;
    Vehicle* leader = nullptr;
    double gap = 1e9;
    for (Vehicle* o : road->vehicles) {
        if (o == v || o->lane != lane) continue;
        if (o->position_on_road <= v->position_on_road) continue;
        const double g = gap_between(v, o);
        if (g < gap) { gap = g; leader = o; }
    }
    return Vehicle::idm_accel(v->current_speed, v->desired_speed_on(*road),
                              leader ? leader->current_speed : 0.0,
                              leader ? std::max(0.0, gap) : 1e9);
}

Vehicle* JunctionArbiter::follower_in_lane(Vehicle* v, int lane) const {
    Vehicle* best = nullptr;
    double best_gap = 1e9;
    if (!v->current_road) return nullptr;
    for (Vehicle* o : v->current_road->vehicles) {
        if (o == v || o->lane != lane) continue;
        if (o->position_on_road >= v->position_on_road) continue;
        const double g = gap_between(o, v);
        if (g < best_gap) { best_gap = g; best = o; }
    }
    return best;
}
```

**Meaning:** `accel_in_lane(v, lane)` answers a *hypothetical*: "if `v` were in `lane`
right now, what would its IDM acceleration be, given whoever's ahead of it in that
lane?" It never actually moves `v` — it's a pure query used to compare lanes.
`follower_in_lane(v, lane)` finds the vehicle immediately *behind* `v`'s position in a
candidate lane — MOBIL's "new follower," the vehicle that would end up behind `v` if
it changed into that lane.

**Why:** MOBIL (Minimizing Overall Braking Induced by Lane changes) needs three
acceleration values to make a decision: how `v` would accelerate in its current lane,
how it would accelerate in the candidate lane, and how the candidate lane's existing
occupant behind that gap would be affected by `v` cutting in front of it. These two
functions supply exactly those probes: `accel_in_lane()` for the first two (called
once with `v->lane`, once with the candidate), `follower_in_lane()` to find who to
check the "politeness" cost against.

**Usage:** both are called exclusively from `solve_lane_changes()`, the next section.

**Check yourself:**
- Why does `accel_in_lane()` never actually change `v->lane` — what would break if it did?

---

### Lines 154–248: `solve_lane_changes()` — MOBIL, and the exact junction-overlap gap

```cpp
void JunctionArbiter::solve_lane_changes(double sim_time) {
    for (auto& vp : vehicles_) {
        Vehicle* v = vp.get();
        Road* road = v->current_road;
        if (!road || road->get_lanes() < 2) continue;
        if (!v->lane_change_ready(sim_time)) continue;

        const std::vector<int> allowed = v->allowed_lanes(road->get_lanes());
        if (allowed.size() < 2) continue;

        const double a_cur = accel_in_lane(v, v->lane);
        const double a_free = Vehicle::idm_accel(v->current_speed,
                                                 v->desired_speed_on(*road), 0.0, 1e9);
        if (v->current_speed >= 1.0 && a_cur >= a_free - kMobilObstruction) continue;

        int best_lane = v->lane;
        double best_gain = kMobilThreshold;

        for (int lane : allowed) {
            if (lane == v->lane || std::abs(lane - v->lane) != 1) continue;

            bool slot_taken = false;
            for (Vehicle* o : road->vehicles) {
                if (o == v || o->lane != lane) continue;
                const double d = std::abs(o->position_on_road - v->position_on_road);
                const double needed =
                    0.5 * (o->length_meters() + v->length_meters()) + kLaneChangeMinGap;
                if (d < needed) { slot_taken = true; break; }
            }
            if (slot_taken) continue;
            ...
        }
        if (best_lane != v->lane) {
            v->set_lane(best_lane, sim_time);
            lane_changes_++;
        }
    }
}
```

**Meaning:** for every vehicle on a multi-lane road that's due for a lane-change
decision (`lane_change_ready`), first checks whether it's even worth considering: if
`v` is moving reasonably fast (`>= 1.0` m/s) *and* its current acceleration is close to
what it would get on a totally free road (within `kMobilObstruction`, 0.4), there's
nothing to gain from weaving — skip it entirely. Otherwise, for each *adjacent*
candidate lane (`std::abs(lane - v->lane) != 1` excludes non-adjacent lanes — no
two-lane jumps in one decision), the **hard gate** runs first: is there any vehicle
`o` in the candidate lane whose *absolute* position difference from `v` is less than
`needed` (half the combined lengths plus `kLaneChangeMinGap`, 1 m)? If so, `slot_taken = true`
and that lane is skipped outright, before any of MOBIL's gain/politeness math runs.
Only if the slot is physically free does the function go on to compute MOBIL's actual
gain calculation (own gain, queued-vehicle incentive, politeness against the new
follower, flip-flop threshold) and possibly pick that lane as `best_lane`.

**Why — this is the concrete origin of "junction overlapping can happen":** the
comment directly above the hard-gate check is unusually explicit about a bug this
code was written to close, and equally explicit that the fix is *narrower* than the
underlying hazard:

> Hard gate first, before any gain math: is the slot physically free? Distance is
> ABSOLUTE, deliberately -- accel_in_lane() only looks at vehicles strictly ahead and
> follower_in_lane() only at ones strictly behind, so a vehicle sitting at the SAME
> position in the target lane (two cars side by side, which is exactly what a queue
> at a stop line produces) is invisible to both, and MOBIL reads the lane as empty.
> That blind spot is what produced observed overlaps of a full car length, with both
> vehicles reporting an identical position_on_road.

Read that carefully: `accel_in_lane()` only searches vehicles with
`position_on_road > v->position_on_road` (strictly ahead), and `follower_in_lane()`
only searches `position_on_road < v->position_on_road` (strictly behind). A vehicle
sitting at *exactly* the same `position_on_road` as `v`, in the candidate lane — which
is precisely what happens when two vehicles queue side-by-side at a stop line — falls
into neither search. MOBIL's own gain calculation, left unchecked, would read that
lane as completely empty and rate the "gain" from moving into it as if nothing were
there, producing an overlap: two vehicles reporting the identical `position_on_road`,
one full car length overlapping the other. `kLaneChangeMinGap` and this hard,
absolute-distance gate are the fix — but notice it is scoped to *this one code path*
(a MOBIL-initiated voluntary lane change). It does not fix the underlying blind spot
in `accel_in_lane()`/`follower_in_lane()` themselves — anything else in the codebase
that calls those two functions inherits the same "invisible if exactly alongside"
gap. And this file's `set_lane()` comment in the constants block says outright:
"`set_lane()` itself does no collision check at all (it only starts the sideways-drift
animation), so nothing else was stopping that" — meaning the *only* thing preventing a
lane-change overlap anywhere in this codebase is this one hard gate, in this one
function, applied only to voluntary MOBIL decisions. Related but separate: this is
lane-change overlap specifically. `solve_junctions()`'s own box-occupancy check
(below) is a *different, coarser* mechanism for junction-crossing conflicts, and it
has its own honest gaps.

**Usage:** called once per tick, from `TrafficSimulator::tick_events_and_lanes()`
(phase 1) — before any vehicle moves this tick, so a lane change decided here is what
phase 2's `lead_in_lane()`/`advance()` calls see for the rest of the tick.

**Check yourself:**
- Precisely which two functions have the "invisible if exactly alongside" blind spot, and why does searching only strictly-ahead / strictly-behind vehicles create it?
- The hard gate here uses an *absolute* distance check rather than calling `gap_between()` (which subtracts vehicle lengths using signed front/back order). Why does that distinction matter when the two vehicles being compared could be at literally the same position?
- The comment says this fix addresses one *specific* code path. Name at least one other place in this file where a vehicle's position is compared to another's using only "strictly ahead" or "strictly behind" logic, and explain why that place could, in principle, have the same blind spot.

---

### Lines 250–464: `solve_junctions()` — right of way, and exactly what "overlap can happen" means at a junction

```cpp
void JunctionArbiter::solve_junctions(
    double dt, double sim_time,
    std::vector<std::pair<Vehicle*, double>>& leaders,
    std::vector<int>& waiting_on) {

    auto is_crossing = [](const Intersection* n) {
        return n && !n->is_roundabout && n->zone != "waypoint";
    };
    auto both_circulating = [](const Road* a, const Road* b) {
        return a && b && a->is_internal && b->is_internal;
    };
    auto circulating_has_priority = [](const Road* mine, const Road* theirs) {
        return mine && theirs && mine->is_internal && !theirs->is_internal;
    };
```

**Meaning:** three small lambdas set the vocabulary for the rest of the function.
`is_crossing` excludes roundabout nodes and "waypoint" zone nodes from needing
right-of-way negotiation at all. `both_circulating`/`circulating_has_priority` handle
roundabout internal roads specially — two vehicles both already *on* the ring never
conflict with each other, and a vehicle on the ring always outranks one merging in
from an approach road.

The big comment above this method (already partially quoted in the file, worth
reading in full in the source) explains the *entire prior design* and why it was
replaced: the old rule grouped every vehicle within a flat approach distance of a
node and had exactly one "win" while the rest stopped; on a real OSM extract where the
median block is close to that same flat distance, "78% of the fleet was permanently
in a group, N-1 of them stopped every tick, and they stopped for each other even when
their paths never crossed." The replacement models what a driver actually does: check
whether the box is physically occupied by cross traffic, and compare arrival *times*
(not distances) against anyone else approaching.

**Meaning (the pre-pass, building `node_occupants_`/`node_approaching_`):**

```cpp
for (std::size_t i = 0; i < vehicles_.size(); ++i) {
    const Vehicle* w = vehicles_[i].get();
    Road* r = w->current_road;
    if (!r) continue;
    const double to_end = r->length - w->position_on_road;
    const bool moving = w->current_speed > kMovingSpeed;

    if (is_crossing(r->target) && r->target->index < node_occupants_.size()) {
        const uint32_t nidx = r->target->index;
        if (to_end < kJunctionZone && (moving || to_end < kJunctionZone - 2.0)) {
            node_occupants_[nidx].push_back({i, r});
        } else if (to_end < junction_approach_) {
            node_approaching_[nidx].push_back(i);
        }
    }
    if (is_crossing(r->source) && w->position_on_road < kJunctionZone &&
        (moving || w->position_on_road < kJunctionZone - 2.0) &&
        r->source->index < node_occupants_.size()) {
        const uint32_t nidx = r->source->index;
        node_occupants_[nidx].push_back({i, r});
    }
}
```

Every vehicle is classified, per junction, as either **occupying** the box (within
`kJunctionZone`, 2 m, of the junction node — either about to arrive at its road's
`target` end, or just departed from its road's `source` end) or **approaching** it
(within `junction_approach_`, the tunable give-way distance, but not yet in the box).
Note the occupancy test isn't purely distance-based: a vehicle within the zone but
*stopped* (`!moving`) only counts as an occupant if it's within an even tighter margin
(`kJunctionZone - 2.0`, effectively 0 m here since `kJunctionZone` is 2.0) — a stopped
vehicle right at the edge of the zone doesn't count as blocking the box.

**Meaning (the decision pass — this is where overlap risk actually lives):**

```cpp
if (!signalized && nidx < node_occupants_.size()) {
    for (const auto& occ : node_occupants_[nidx]) {
        if (occ.vehicle_idx == i) continue;
        if (occ.road->base_index == my_base_idx) continue;
        if (both_circulating(road, occ.road)) continue;
        if (circulating_has_priority(road, occ.road)) continue;
        if (occ.road == my_next) continue;
        yield = true;
        if (vehicles_[occ.vehicle_idx]->current_speed < kStoppedSpeed) {
            blocker = vehicles_[occ.vehicle_idx].get();
        }
        break;
    }
}
```

Rule (a): if *any* occupant of the junction box is on a *different* street (different
`base_index`) than `v`, and none of the exemptions apply (same roundabout, ring
outranking approach, or the occupant is on the road `v` itself is about to turn onto),
`v` must yield. It checks only the **first** conflicting occupant found (`break`),
not every occupant, and not whether `v`'s own approach path geometrically intersects
that occupant's path at all — it is a coarse "is the box occupied by someone from a
different street" test, not a precise trajectory-intersection test.

Rule (b), the time-gap comparison, only fires if rule (a) didn't already force a
yield — it compares `v`'s estimated arrival time against every *approaching* (not
yet in the box) vehicle from a conflicting street, using an impatience bonus that
grows with how long `v` has already waited (`zone_wait()`), capped at `kZoneBonusCap`
(3 s), specifically so that a chronically-starved approach eventually wins a slot
rather than deadlocking forever against a busier one.

Rule (c), "do not block the box," checks whether the space *downstream* of the
junction (either the immediate leader, or the tail of the next road on `v`'s route) is
stopped and too close for `v` to fully clear the junction if it entered now — if so,
`v` waits at the line rather than risk stalling mid-crossing.

**Why — the honest limit of what's checked:** rule (a) is a real, working check — it
does prevent a vehicle from entering a box that's already got a cross-street vehicle
sitting in it, at the granularity of "is anyone from a different street physically in
the `kJunctionZone` radius." But that granularity is exactly the gap. `kJunctionZone`
is a flat 2 m radius around the junction *node point*, not a shape derived from each
road's actual width or the junction's real geometry, and the occupancy test is purely
"index of the road you're on differs" plus a handful of named exemptions — it has no
concept of which specific paths through the junction actually cross in space. Two
vehicles from different streets that are both, say, turning in directions that
wouldn't geometrically conflict (a right turn from one approach and a right turn from
a perpendicular approach, depending on the junction's real shape) are still forced
into the same yield/proceed binary as two vehicles whose paths would collide head-on
— the model can't tell those cases apart, because it never models path geometry inside
the box at all, only "is the box occupied by someone not from my street." Combine that
with the lane-change hard gate's own admitted blind spot (vehicles at *identical*
`position_on_road` being invisible to strictly-ahead/strictly-behind searches, covered
above) and with the fact that `kJunctionZone` occupancy for a *stopped* vehicle needs
an even tighter margin to register at all (the `moving || to_end < kJunctionZone - 2.0`
condition) — there are real, identifiable gaps between "yield if someone else is in
the box" and "guarantee no two vehicles ever occupy overlapping space at a junction."
This is the concrete code behind "junction overlapping can happen": the check that
exists is real and does real work, but it is a coarse presence test at a fixed radius
with a same-street exemption and a handful of roundabout special cases — not a
geometric collision check.

**Usage:** called once per tick from `tick_leads_and_physics()`, mutating `leaders`
in place for any vehicle told to yield (its leader is replaced with a synthetic "stop
line" position so IDM brakes it smoothly rather than the vehicle freezing instantly),
and mutating `waiting_on` when the thing being yielded to is genuinely stationary.

**Check yourself:**
- Rule (a) breaks out of its occupant loop after the *first* conflicting occupant found. What does that imply about how it would behave if two different-street vehicles were both occupying the box at once — does `v` evaluate its conflict against both, or only whichever happens to be first in `node_occupants_[nidx]`?
- `kJunctionZone` is a flat 2 m radius from the junction node, the same value regardless of how many lanes or how wide the actual junction is. What kind of real intersection (wide, multi-lane) would this radius most plausibly be too small for, and why?
- The occupancy rule is "different street => yield," not "paths actually cross => yield." Describe a turning-movement scenario at a four-way junction where two vehicles from different streets would not, in reality, ever occupy the same physical space, but this code would still force one of them to yield.

---

### Lines 466–527: `find_gridlock_cycle_leaders()` and `break_cycles()` — M7 stage 1, detection by graph structure

```cpp
std::vector<std::size_t> find_gridlock_cycle_leaders(
    const std::vector<int>& waiting_on, const std::vector<double>& stuck_times) {
    std::vector<std::size_t> leaders;
    const std::size_t n = waiting_on.size();
    std::vector<int> mark(n, 0);
    int stamp = 0;

    for (std::size_t s = 0; s < n; ++s) {
        if (mark[s]) continue;
        ++stamp;
        int u = static_cast<int>(s);
        while (u >= 0 && !mark[u]) {
            mark[u] = stamp;
            u = waiting_on[u];
        }
        if (u >= 0 && mark[u] == stamp) {
            std::size_t best = static_cast<std::size_t>(u);
            for (int w = waiting_on[u]; w != u; w = waiting_on[w]) {
                if (stuck_times[w] > stuck_times[best]) {
                    best = static_cast<std::size_t>(w);
                }
            }
            leaders.push_back(best);
        }
    }
    return leaders;
}
```

**Meaning:** this is a single O(n) pass with a "stamp" per walk, exploiting the fact
that `waiting_on` is a *functional* graph — every vehicle waits on at most one other
(out-degree ≤ 1), so from any starting vehicle there's exactly one path forward, never
a branch. Walking from an unvisited vehicle `s`, following `waiting_on[u]` until
either running off the end (`u == -1`, nobody waiting, no cycle down this path) or
landing on an already-`mark`ed node: if that mark carries *this walk's* stamp, the
walk has closed a cycle — everyone visited since `mark[u]` was first stamped this
round is part of a ring. If the mark belongs to an *earlier* walk's stamp, this path
merged into ground already covered by a previous search, and any cycle there (if one
exists) was already reported by that earlier walk. Within a found cycle, it picks the
member with the longest `stuck_times[]` as the one to release — the comment explains
this is sufficient because releasing one frees the next, and so on around the ring.

**Why:** the long comment above this function (already quoted in full in the header
section — worth restating the key line here) makes the "reactive" framing precise:
"The old rule was 'stopped for 10 seconds means deadlocked.' That cannot tell a queue
from a deadlock... Time is the wrong question. The right one is whether the wait-for
relation has a cycle in it." This is real, structural cycle detection — not a
stopwatch heuristic — and it's a genuine improvement over "stopped too long ⇒
deadlocked," which would misfire on a 40th-in-line vehicle at a busy light exactly the
way a too-loose `kWaitEdgeGap` misfired earlier. But notice what it still is: it only
*ever runs on the state that already exists* — a cycle in `waiting_on` is, by
definition, a set of vehicles already stopped and already blocking each other. There
is no mechanism anywhere that refuses to let a vehicle enter a configuration that
would create such a cycle; the graph is built by observing already-stopped vehicles
(`tick_leads_and_physics()`'s `kWaitEdgeGap`/`kStoppedSpeed` check), and this function
only ever looks for cycles *after* they exist, on a 2-second cadence, not continuously.
Structurally sound detection is still detection, not prevention.

```cpp
void JunctionArbiter::break_cycles(const std::vector<int>& waiting_on, double sim_time) {
    std::vector<double> stuck(vehicles_.size());
    for (std::size_t i = 0; i < vehicles_.size(); ++i) {
        stuck[i] = vehicles_[i]->stuck_time;
    }
    for (std::size_t leader : find_gridlock_cycle_leaders(waiting_on, stuck)) {
        Vehicle* v = vehicles_[leader].get();
        if (v->is_forced(sim_time)) continue;
        v->grant_forced_clearance(sim_time + kForcedGrant);
        forced_throughs_++;
        controller_.log_line("Gridlock broken: " + v->id + " forced through");
    }
}
```

**Meaning:** collects every vehicle's current `stuck_time` into a plain array, calls
the free function above, and for each returned cycle-leader index, grants that
vehicle `kForcedGrant` (8) seconds of forced clearance — presumably meaning it's
allowed to proceed through a junction it would otherwise have to yield at, per
`v->is_forced(sim_time)` being checked as a bypass condition in `solve_junctions()`'s
own `if (v->ignores_lights() || v->is_forced(sim_time)) continue;` line — and logs a
human-readable line via the controller.

**Why:** this is the *resolution* half of M7 stage 1 — once a cycle is found, exactly
one vehicle per cycle gets forced through, which the earlier comment says is
sufficient because freeing one link in a ring frees the next.

**Usage:** called from `TrafficSimulator::tick_leads_and_physics()` only every
`kCycleCheckPeriod` (2 s), never continuously — confirmed already in the
`TrafficSimulator.cpp` walkthrough above.

**Check yourself:**
- Why does the "at most one" (out-degree ≤ 1) property of `waiting_on` make single-pass, no-recursion cycle detection possible here, where an arbitrary graph would need something more expensive (e.g. Tarjan's algorithm)?
- The comment explicitly rejects "stopped for 10 seconds" as a deadlock signal. Describe a scenario (already hinted at in the comment) where that old rule would misfire on traffic that is *not* actually deadlocked.
- Is `find_gridlock_cycle_leaders()` itself "reactive" or "preventive," strictly by the definition used elsewhere in this module — and why does being algorithmically sound (cycle detection, not a stopwatch) not change that classification?

---

### Lines 529–548: `solve_gridlock()` — M7 stage 2, the last-resort rescue, and its own admitted cost

```cpp
void JunctionArbiter::solve_gridlock(double sim_time) {
    (void)sim_time;
    for (auto& v : vehicles_) {
        if (v->stuck_time <= kStuckRescue) continue;
        if (v->get_route_index() + 1 >= static_cast<int>(v->get_route().size())) continue;
        if (v->rescue_to(static_cast<std::size_t>(v->get_route_index()) + 1)) {
            rescues_++;
        }
    }
}
```

**Meaning:** for every vehicle stuck longer than `kStuckRescue` (45 s) that still has
a next road on its route, this attempts `v->rescue_to(...)` — teleporting the vehicle
forward to the next road on its route, entirely bypassing whatever's holding it up.
`rescues_` is only incremented when `rescue_to()` actually succeeds — the comment
explains why that check matters: `rescue_to()` internally refuses when the target
slot is occupied and deliberately leaves `stuck_time` untouched so the next tick tries
again, so before this success check existed, a vehicle wedged for good would book one
"rescue" every tick forever — the comment cites a measured "19212 of them in 9
sim-seconds," which "made the one statistic whose job is to say 'the traffic model is
wrong' completely unreadable."

**Why:** the function's own doc comment in the header calls this "the escape hatch of
last resort... if it runs often, the traffic model is wrong and this is papering over
it." This is the bluntest possible statement, in the code itself, of what reactive
gridlock handling costs: stage 1 (`break_cycles`) is at least principled — it only
fires on a proven cycle in the wait-for graph. Stage 2 fires on nothing more
structured than "stuck for 45 real-scaled seconds, still has somewhere to go" — no
attempt to understand *why* it's stuck, no cycle proof, just teleportation. It exists
specifically to catch the vehicles stage 1's cycle detection *can't* name — a vehicle
stuck for reasons other than a clean cycle in `waiting_on` (e.g. permanently blocked by
something outside the wait-for graph's model entirely) has no other relief mechanism
in this codebase.

**Usage:** called every tick (not throttled like `break_cycles()`) from
`tick_leads_and_physics()`, right after `solve_junctions()` and before the cycle-check
throttle gate.

**Check yourself:**
- Why does `rescues_` only increment on a *successful* `rescue_to()` call rather than every time the 45-second threshold is crossed, and what statistic would have been useless without that distinction (the comment gives a specific measured number)?
- The header's own comment calls this function "papering over" a wrong traffic model if it fires often. In your own words, what would a *preventive* alternative to this function look like — what would have to be checked, and when, to make stage 2 unnecessary?

---

**Check yourself (whole file):**
- `JunctionArbiter` implements three named responsibilities: MOBIL lane changes, junction right of way, and gridlock relief. For each of the three "known limitations" this module was asked to engage with — junction overlap, reactive gridlock, single-threaded tick loop — name the specific function(s) in this file most directly responsible, and in one sentence say what a fix would have to add or change.
- `find_gridlock_cycle_leaders()` is deliberately a free function for testability, while every other solver method is a `JunctionArbiter` member. If you were asked to add a genuinely *preventive* gridlock check (run before a vehicle is allowed to enter a junction box, not after it's already stuck), would that new check belong as a free function or a member — and why, given the reasoning already used to justify the existing split?

---

## Where this leaves the three limitations

Concretely, in this module's own code:

- **"Junction overlapping can happen"** — `JunctionArbiter::solve_lane_changes()`'s
  hard gate (`kLaneChangeMinGap`) fixes one specific, documented overlap cause (two
  vehicles at identical `position_on_road` being invisible to `accel_in_lane()`/
  `follower_in_lane()`'s strictly-ahead/strictly-behind searches) — but only for
  voluntary MOBIL lane changes. `solve_junctions()`'s box-occupancy check
  (`node_occupants_`, rule (a)) is a coarse "is anyone from a different street within
  a flat 2 m radius of the node" test, not a geometric path-conflict check — it has no
  model of which specific turning movements at a junction actually cross in space.
- **"Gridlocks are handled reactively rather than prevented"** — the `waiting_on`
  wait-for graph is built in `TrafficSimulator::tick_leads_and_physics()` by
  observing vehicles that are *already* stopped near each other; `JunctionArbiter::break_cycles()`
  (M7 stage 1) looks for cycles in that graph only every 2 seconds, and
  `JunctionArbiter::solve_gridlock()` (M7 stage 2, its own doc comment calls it "the
  escape hatch of last resort") teleports any vehicle stuck past 45 seconds that
  stage 1 couldn't resolve. Nothing in either mechanism, or anywhere in `solve_junctions()`,
  refuses in advance to let a vehicle enter a configuration that would deadlock.
- **"The tick loop is single-threaded"** — `TrafficSimulator::tick()` and every phase
  it calls (`tick_events_and_lanes`, `tick_leads_and_physics`, `tick_cleanup_and_spawn`,
  `tick_statistics`, `tick_history_capture`) are plain, sequential `for` loops over
  `vehicles_`/`graph_->edges`, executed by whichever single thread calls `tick()`,
  all inside one `std::lock_guard<std::mutex> lock(mtx_)` held for the whole call. The
  other threads in the process (`publish_loop()`, and the HTTP/WebSocket server
  threads implied by `find_package(Threads REQUIRED)` in `CMakeLists.txt`) never
  execute simulation logic themselves — they read published snapshots or briefly
  contend for `mtx_` for request-scoped mutations, never splitting a single tick's
  work across more than one thread.
