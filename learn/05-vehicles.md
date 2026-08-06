# Module 05 — Vehicles

**Files covered:**
- [`cpp_backend/include/vehicles/Vehicle.hpp`](../cpp_backend/include/vehicles/Vehicle.hpp)
- [`cpp_backend/src/vehicles/Vehicle.cpp`](../cpp_backend/src/vehicles/Vehicle.cpp)
- [`cpp_backend/include/vehicles/VehicleTypes.hpp`](../cpp_backend/include/vehicles/VehicleTypes.hpp)
- [`cpp_backend/src/vehicles/VehicleTypes.cpp`](../cpp_backend/src/vehicles/VehicleTypes.cpp)

This module is where the car-following model lives, where the "no continuous
lateral position" limitation from the syllabus has a concrete address, and
where the project's first hint of MOBIL-style lane changing shows up (the
full MOBIL decision itself is computed in `TrafficSimulator`, covered in
`learn/07-simulation-core.md` — what you'll see here is the state `Vehicle`
exposes so that decision has something to act on). `Vehicle` is an abstract
base class; `VehicleTypes.hpp/.cpp` is where `Car`, `Bus`, `EmergencyVehicle`
and `Motorbike` plug numbers and rule overrides into it, using the Factory
pattern to centralize construction. Route data comes from
`learn/04-routing.md`'s `PathFinding` strategies, and every `Road`/`Intersection`
reference here is the graph model from `learn/02-core-graph-model.md`.

---

## `cpp_backend/include/vehicles/Vehicle.hpp`

### Lines 1–15: includes and forward declarations

```cpp
#include "core/Road.hpp"
#include "routing/PathFinding.hpp"
#include "utils/Observer.hpp"

class Intersection;
class Road;
class Building;
class TrafficGraph;
```

**Meaning:** `Vehicle.hpp` fully includes `Road.hpp` (it stores a `Road*` and
calls `Road` methods like `add_vehicle`), `PathFinding.hpp` (it owns a
`std::unique_ptr<PathFinding>`, so the compiler needs the complete type to
know its size for `unique_ptr`'s deleter), and `Observer.hpp` (`Vehicle`
inherits from `IObserver`, so it needs that base class's full definition).
`Intersection`, `Building`, and `TrafficGraph` are only ever referenced
through pointers in this header, so forward declarations are enough — no
full include needed, which keeps compile-time dependencies from ballooning.

**Why:** this is the standard C++ discipline of "include what you use by
value or by inheritance, forward-declare what you only touch by pointer or
reference." `Road` is a genuine dependency (not just forward-declared)
because `Vehicle` calls `Road::add_vehicle` and reads `Road::length` inline
in `.cpp` — those need the complete `Road` type available.

**Usage:** Anything that includes `Vehicle.hpp` transitively pulls in
`Road.hpp`, `PathFinding.hpp`, and `Observer.hpp` — worth knowing before
adding new dependencies here, since this header is included from the
simulator core, the WebSocket layer, and every test file that touches a
vehicle.

**Check yourself:**
- Why does `TrafficGraph` get a forward declaration instead of a full `#include`, when `Road` gets a full include?
- What would break if `PathFinding.hpp` were only forward-declared instead of included?

---

### Lines 19–27: the class and the `State` enum

```cpp
class Vehicle : public IObserver
{
public:
    enum class State { Planning, Moving, WaitingAtLight, Blocked, Arrived };
```

**Meaning:** `Vehicle` publicly inherits `IObserver` — every vehicle is an
Observer in the Observer pattern wired up in `utils/Observer.hpp`. `State`
is a scoped enum (`enum class`, so `State::Moving` rather than a bare
`Moving` leaking into the surrounding scope) with five values describing why
a vehicle is or isn't moving right now.

**Why:** the comment directly above this in the source is explicit about the
motivation: "A car held at a red light and a car wedged in a deadlock look
identical from the outside; only the simulator knows which is which." Without
this enum, the frontend (or a test) would have to infer state from
`speed == 0`, which is ambiguous — a stopped-at-light car and a gridlocked
car both report `0.0`. Making the *reason* a first-class, queryable field is
what lets `/api/state` report something more useful than a raw number.

**Usage:** `state_` is set inside `advance()` (see below) before the physics
runs each tick, and read back through the `state()` accessor. It's part of
the binary snapshot format the frontend consumes — see `learn/09-server-and-entrypoint.md`
for how vehicle state gets serialized.

**Check yourself:**
- Why is `State` an `enum class` instead of a plain `enum`?
- What's the practical difference between `Blocked` and `WaitingAtLight` from the simulator's point of view?

---

### Lines 29–33: core position and motion fields

```cpp
std::string id;
uint32_t num_id = 0;        // Fast integer ID for zero-copy binary streaming
Road* current_road; // The road the vehicle is on
double position_on_road; // The distance traveled by the vehicle, measured from the start of the road
double current_speed; // Current speed of vehicle
```

**Meaning:** `id` is the vehicle's string identifier (e.g. `"car42"`).
`num_id` is a redundant integer form of the same identity, called out as
existing purely for "zero-copy binary streaming." `current_road` is a raw,
non-owning pointer — `Vehicle` doesn't own the `Road`, the `Graph` does (see
`learn/02-core-graph-model.md`). `position_on_road` is a *scalar* distance
along the road's length, and `current_speed` is the vehicle's scalar speed.

**Why:** this is the first concrete evidence of the "vehicles are locked to
discrete lanes" limitation from the project's known-limitations list.
Position along a road is one number, not a 2D or even a (distance, lateral)
pair here — there's no `x`/`y` sub-lane offset on this line. A vehicle's
*position* on its road is one-dimensional; which lane it's in is a wholly
separate integer field (`lane`, next chunk). `num_id` existing alongside
`id` is a pure performance concession: streaming a `uint32_t` per vehicle
per tick over a WebSocket is cheaper than streaming a variable-length
string, but the string version is kept because it's what the rest of the
backend (route storage, logs, notifications) addresses vehicles by.

**Usage:** `current_road` is what every road-local computation (leader
lookup, `entry_slot_free`, IDM) reads to know "which road's vehicle list is
this vehicle in." `position_on_road` is compared against `current_road->length`
every tick in `advance()` to detect a road handover.

**Check yourself:**
- Why is `current_road` a raw pointer rather than a `shared_ptr` or `unique_ptr`?
- If `Vehicle` wanted to support continuous lateral position (an actual x/y within the lane), which field here would need to change, and what else would immediately need to change alongside it?

---

### Lines 35–42: IDM and lane-change state block

```cpp
double idm_factor{1.0};             // per-vehicle speed personality (0.8 .. 1.0)
double lateral_offset{0.0};         // current sideways offset from the lane centre
double target_lateral_offset{0.0};  // where the lane change is heading
int lane{0};                        // current lane, 0 is the innermost
double stuck_time{0.0};             // seconds spent standing still
double last_gap{1e9};               // gap the last advance() actually used
std::size_t sim_index = 0;          // Fast zero-overhead index in simulator fleet vector
```

**Meaning:** `idm_factor` scales this specific vehicle's desired speed
(0.8–1.0), giving each vehicle a slightly different "personality" instead of
every car of the same type driving identically. `lateral_offset` and
`target_lateral_offset` are a *cosmetic* sideways drift used only to animate
a lane change smoothly on screen — they ease from one lane's centre-line
offset back to zero, they do not represent a continuous position the physics
or collision logic reasons about. `lane` is the actual, discrete lane index
the vehicle occupies — an `int`, not a float, so a vehicle is always
*exactly* in lane 0, or lane 1, or lane 2, never "60% into lane 1." `stuck_time`
accumulates while the vehicle is nearly stationary, feeding into gridlock
detection. `last_gap` records the gap `advance()` actually used last tick
(diagnostic/telemetry). `sim_index` is a cache-friendly index into the
simulator's flat fleet vector, avoiding a lookup.

**Why:** this is the second, sharper piece of evidence for the lane-locking
limitation: `lane` is an `int`. All of the collision-avoidance, gap-finding,
and following logic in `Vehicle.cpp` and `JunctionArbiter.cpp` (see
`entry_slot_free` below) key off this integer, comparing `o->lane != lane`
to decide whether two vehicles are even in contention for the same space.
`lateral_offset` looks like it might be the continuous position that would
fix this, but read the comment carefully — it's driven only by
`set_lane()`'s "animate across" logic (see the `.cpp` section) and eased
back to `0.0`, not steered by any driving decision. A vehicle mid-lane-change
is still, for every physics and safety purpose, "in lane `X`" or "in lane
`Y`" — never a real intermediate lateral state a neighboring vehicle's gap
calculation accounts for.

**Usage:** `lane` is read by `entry_slot_free()`, by `allowed_lanes()`
overrides in `VehicleTypes.hpp`, and by the simulator's per-tick leader
search (`learn/07-simulation-core.md`). `lateral_offset`/`target_lateral_offset`
are read only by the rendering/serialization path — grep for them outside
`Vehicle.cpp` and you'll find they don't participate in any distance or gap
computation.

**Usage — check yourself:**
- Given that `lateral_offset` exists but doesn't affect gap calculations, what would have to change in `entry_slot_free()` (see the `.cpp` file) for a lane change to be judged unsafe *during* the drift, rather than only judged at the instant `lane` flips?
- Why start `idm_factor` at a range of `0.8..1.0` instead of allowing values above `1.0`?

---

### Lines 44–48: constructor and destructor

```cpp
Vehicle(std::string id, double max_speed, double max_accel, double max_decel);
virtual ~Vehicle() = default;
```

**Meaning:** The constructor takes the vehicle's id and its three speed
limits by value (`id` is a `std::string` taken by value, which lets the
caller either copy or move an existing string in cheaply). The destructor is
`virtual` — required because `Vehicle` is a polymorphic base class — and
`= default` because `Vehicle` doesn't own any resources that need custom
cleanup beyond what its members (a `unique_ptr<PathFinding>`, `vector`s,
`string`s) already handle themselves.

**Why:** `virtual ~Vehicle()` is non-negotiable the moment `VehicleFactory::create`
(covered later) returns a `std::unique_ptr<Vehicle>` pointing at a `Car`,
`Bus`, `EmergencyVehicle`, or `Motorbike`. Without a virtual destructor,
deleting through a base-class pointer is undefined behavior — the derived
class's destructor (and its members' destructors) might never run.

**Usage:** Every vehicle in the simulator's fleet is stored and destroyed
through this base-class interface, so this one keyword is what keeps the
whole `VehicleFactory` design safe.

**Check yourself:**
- What concrete bug could occur at runtime if `~Vehicle()` were *not* virtual, given how `VehicleFactory::create()` returns vehicles?

---

### Lines 50–52: the IDM math surface

```cpp
static double idm_accel(double v, double v0, double v_lead, double gap);
double desired_speed_on(const Road& road) const;
```

**Meaning:** `idm_accel` is `static` — it doesn't touch `this`, it's a pure
function of its four arguments (current speed, desired speed, leader's
speed, and gap to the leader), returning an acceleration. `desired_speed_on`
is a normal (non-static) `const` member function that computes this specific
vehicle's target speed on a given `Road`.

**Why:** making `idm_accel` `static` and free of any vehicle-specific state
is deliberate — it's the reusable core of the Intelligent Driver Model (IDM),
a car-following model (Treiber et al.) where acceleration is a function of
your own speed, your desired speed, your leader's speed, and the gap between
you. Keeping it static and stateless means it's trivially unit-testable in
isolation (see `tests/unit_tests/` for exactly that) without constructing a
whole `Vehicle`.

**Usage:** `desired_speed_on()` folds together the road's speed limit, this
vehicle type's `speed_factor()`, and this vehicle's individual `idm_factor`
into one number, which then feeds as the `v0` argument into `idm_accel()`
inside `advance()`.

**Check yourself:**
- Why is `idm_accel` a `static` member instead of a free function outside the class entirely?
- What three sources of "speed" does `desired_speed_on` combine (see the `.cpp` definition for the exact formula)?

---

### Lines 54–67: the polymorphic type hooks

```cpp
virtual std::string type() const = 0;
virtual double speed_factor() const { return 1.0; }
virtual double length_meters() const { return 4.5; }
virtual double width_meters() const { return 1.8; }
virtual double congestion_sensitivity() const { return 1.0; }
virtual bool ignores_lights() const { return false; }
virtual double lane_politeness() const { return 0.5; }

virtual std::vector<int> allowed_lanes(int total_lanes) const {
    std::vector<int> lanes;
    for (int i = 0; i < total_lanes; ++i) lanes.push_back(i);
    return lanes;
}
```

**Meaning:** `type()` is pure virtual (`= 0`) — `Vehicle` cannot be
instantiated directly, only through a concrete subclass that supplies a
type name. Everything else is a *virtual function with a default
implementation*: `speed_factor` defaults to `1.0` (no adjustment),
`length_meters`/`width_meters` default to typical car dimensions,
`congestion_sensitivity` defaults to `1.0` (fully affected by road
congestion), `ignores_lights` defaults to `false`, `lane_politeness` defaults
to `0.5` (a middling MOBIL politeness factor), and `allowed_lanes` defaults
to "every lane on the road is fine" (it builds `[0, 1, ..., total_lanes-1]`).

**Why:** this is the Template/Hook Method flavor of polymorphism this
codebase uses throughout: a base class defines *sensible defaults* for
every behavior, and a subclass only overrides the ones where it differs.
`Car` (see `VehicleTypes.hpp`) overrides none of these — it *is* the
default vehicle. This keeps each concrete vehicle class down to just the
handful of lines where it actually differs from baseline, instead of every
subclass repeating a full set of values.

**Usage:** These are the exact hooks `advance()` calls into: `speed_factor()`
and `congestion_sensitivity()` feed `desired_speed_on()`; `length_meters()`
feeds `entry_slot_free()`'s clearance math and the stop-line calculation;
`ignores_lights()` is checked directly in the traffic-light branch of
`advance()`; `lane_politeness()` is read by the simulator's MOBIL
lane-change incentive check (`learn/07-simulation-core.md`); `allowed_lanes()`
is consulted every time a vehicle needs to pick a lane on a new or changed
road (`transfer_to_road`, `rescue_to`, and the road-handover step of
`advance()`).

**Check yourself:**
- Why is `type()` pure virtual while every other hook here has a default body?
- Trace one hook (say `ignores_lights()`) from its declaration here to where it's actually called in `advance()` in the `.cpp` file.

---

### Lines 69–98: routing entry points and getters

```cpp
void request_route(TrafficGraph* graph, Intersection* start, Intersection* target);
void recompute_route(TrafficGraph* graph, Intersection* target);
bool has_reached_destination() const;
std::string get_id() const;
Road* get_current_road() const;
Intersection* get_destination() const;
double get_position_on_road() const;
const std::vector<std::string>& get_travel_history() const;
const std::vector<Road*>& get_route() const;
int get_route_index() const;
double get_current_speed() const;
double get_max_speed() const;
double get_acceleration() const;
double get_max_acceleration() const;
double get_max_deceleration() const;
std::size_t last_nodes_explored() const;
```

**Meaning:** `request_route` is the initial planning call — given the
graph and a start/target `Intersection`, it computes and stores a route.
`recompute_route` is its counterpart for mid-trip replanning. Everything
after that is a `const` accessor exposing private state: the route as a
list of `Road*`, where in that list the vehicle currently is
(`get_route_index`), speed/acceleration figures, and `last_nodes_explored()`
which forwards to the underlying `PathFinding` strategy's own counter (see
`learn/04-routing.md`).

**Why:** note the comment removed between `request_route` and
`recompute_route` in the source — `step_by_step()` and
`react_to_congestion()` used to exist here and were deleted because
`advance()` (covered below) superseded them and nothing called them anymore.
That's worth internalizing as this codebase's convention: dead methods are
removed outright rather than left "just in case," so if you're reading old
notes or a stale diagram that mentions `step_by_step()`, the current code no
longer has it.

**Usage:** `request_route`/`recompute_route` are how `TrafficSimulator`
(spawn logic) and the rerouting machinery connect a `Vehicle` to the
`PathFinding` strategies of `learn/04-routing.md`. The getters are what the
WebSocket/state-serialization layer and tests read to inspect a vehicle
without touching its private fields directly — standard encapsulation.

**Check yourself:**
- Why keep `request_route` and `recompute_route` as two separate methods rather than one method with a "first time?" boolean flag?
- What does a non-empty return from `get_travel_history()` tell you that `get_route()` alone cannot?

---

### Lines 100–104: lane accessors and the destination setter

```cpp
int get_lane() const { return lane; }
double get_lateral_offset() const { return lateral_offset; }
void set_destination(Intersection* dest) { destination_ = dest; }
```

**Meaning:** Two trivial inline getters exposing the discrete `lane` and the
cosmetic `lateral_offset` field discussed above, plus an inline setter for
`destination_`.

**Why:** these are declared and defined right here in the header (not out of
line in the `.cpp`) because they're one-liners with no logic worth hiding —
a very ordinary C++ convention: implement trivial accessors inline so the
compiler can freely inline them at call sites, and keep anything with real
control flow (`advance()`, `request_route()`, etc.) out-of-line in the `.cpp`.

**Usage:** `get_lane()` and `get_lateral_offset()` are exactly what the
serialization layer reads to tell the frontend where to draw a vehicle
on screen — confirming again that "lane" is the vehicle's real spatial
identity, and "lateral offset" is purely a rendering nicety layered on top.

**Check yourself:**
- Why might a codebase choose to inline trivial getters/setters in the header instead of declaring them here and defining them in the `.cpp`?

---

### Lines 106–132: the "hooks used by TrafficSimulator" block

```cpp
// A Vehicle can only see its own road, so anything needing a global view
// (the lane next door, the queue on the next road, right of way) is
// decided by the simulator and handed in through these.

void onNotify(const TrafficNotification& notification) override;
void request_reroute() { needs_reroute_ = true; }
bool remaining_route_contains(const std::string& road_id) const;
bool consume_reroute_flag();

void set_algorithm(const std::string& name);
const std::string& algorithm() const { return algorithm_; }
void set_cost_mode(CostMode mode);
```

**Meaning:** The comment states the section's architectural rule plainly: a
`Vehicle` only ever knows about its own road, never the wider network state,
so anything that requires a global view is computed by `TrafficSimulator`
and *handed in* through one of these hook methods, rather than `Vehicle`
reaching out to find it itself. `onNotify` is the `IObserver` override — the
`TrafficNotification` type and the Observer wiring itself belongs to
`learn/06-events-observer.md`. `request_reroute()` is a one-line inline
setter for a "please replan" flag. `remaining_route_contains` checks whether
a given road id still appears in the *unfinished* portion of the route.
`consume_reroute_flag` is a read-and-clear accessor. The last three lines
are the Strategy-pattern hooks: `set_algorithm` swaps which `PathFinding`
implementation this vehicle uses, `algorithm()` reports the current choice,
and `set_cost_mode` forwards to the strategy's own cost-mode setter.

**Why:** the "only flags a reroute" comment on `onNotify` in the source
(right above its declaration) explains the design: an Observer callback can
fire mid-notification, while the graph or another vehicle's state might be
in flux, so `onNotify` does the *minimum safe thing* — flip a boolean — and
defers the actual, more invasive recomputation to a point the simulator
controls (the next tick). This is a common Observer-pattern discipline:
keep the notification handler cheap and side-effect-light, and let the
subject (here, the simulator's tick loop) decide when it's safe to act on
what was flagged.

**Usage:** `TrafficController` (Observer subject, `learn/06-events-observer.md`)
calls `onNotify` on every vehicle it's told to notify. `TrafficSimulator`'s
tick loop calls `consume_reroute_flag()` and, if true, calls
`recompute_route()`. `set_algorithm`/`set_cost_mode` are called from the API
layer when a client requests a specific routing strategy per vehicle.

**Check yourself:**
- Why does `onNotify` only set a flag instead of calling `recompute_route()` directly?
- What's the difference in scope between `remaining_route_contains()` and `get_route()` — why does the "remaining" qualifier matter?

---

### Lines 134–185: simulator-driven movement, MOBIL hooks, and gridlock relief

```cpp
void advance(double dt, const Vehicle* lead, double gap, double sim_time);
double travel_time() const { return travel_time_; }

int prev_lane() const { return prev_lane_; }
double prev_lane_blocked_until() const { return prev_lane_blocked_until_; }
bool lane_change_ready(double sim_time) const;
void set_lane(int new_lane, double sim_time);

void add_zone_wait(double seconds) {
    zone_wait_ = std::min(kMaxZoneWait, zone_wait_ + seconds);
}
double zone_wait() const { return zone_wait_; }
bool is_forced(double sim_time) const { return sim_time < forced_until_; }
void grant_forced_clearance(double until_sim_time) { forced_until_ = until_sim_time; }

void transfer_to_road(Road* new_road, double progress_meters);
State state() const { return state_; }
bool rescue_to(std::size_t new_index);
int recalculations() const { return recalculations_; }

bool is_route_dirty() const { return route_dirty_; }
bool can_recompute() const { return recompute_cooldown_ <= 0.0; }
void mark_route_dirty() { route_dirty_ = true; }
void clear_route_dirty() { route_dirty_ = false; }
```

**Meaning:** `advance(dt, lead, gap, sim_time)` is the single per-tick
movement entry point — the comment above it in the source is explicit:
"Unlike `step_by_step()` this does not look for a leader itself, so the
simulator can point it at a vehicle in another lane or on the next road."
The simulator does the (global) work of finding who's ahead, and `advance()`
does the (local) work of reacting to it. `travel_time()` reports accumulated
travel seconds. `prev_lane()`/`prev_lane_blocked_until()`/`lane_change_ready()`/
`set_lane()` are the MOBIL-adjacent lane-change interface — the comment
above them in the source says exactly what they're for: "multi-lane, driven
by the simulator's MOBIL solver." `add_zone_wait`/`zone_wait`/`is_forced`/
`grant_forced_clearance` implement impatience-based gridlock relief at
junctions. `transfer_to_road` moves a vehicle onto a new `Road` object
(used when live map editing splits a road under it). `state()` reports the
`State` enum discussed earlier. `rescue_to` is the "last resort" deadlock
breaker. The remaining four are the dirty-route/cooldown bookkeeping flags
used to gate `recompute_route()` calls.

**Why — the MOBIL connection:** MOBIL (Minimizing Overall Braking Induced by
Lane changes, Kesting/Treiber/Helbing) is a lane-change decision model built
on top of a car-following model like IDM: a vehicle considers changing lanes
by checking (1) an *incentive* — would changing make things enough better
for itself, weighted against how much worse it makes things for the vehicle
that would now be behind it, scaled by a *politeness factor* — and (2) a
*safety* criterion — is the gap in the target lane large enough not to force
a hard brake on whoever's back there. This file doesn't implement that
decision itself — the incentive/safety comparison is computed in
`TrafficSimulator` (see the `kLaneWidth`/lane-selection logic referenced in
`learn/07-simulation-core.md`). What lives here is the *mechanism* the
decision acts through: `lane_politeness()` (declared on `Vehicle`, seen
above) supplies the per-vehicle-type politeness weight MOBIL's incentive
term needs; `lane_change_ready()` gates whether a change is even eligible
right now (not mid-drift, cooldown expired); `set_lane()` executes a change
once decided; `prev_lane_`/`prev_lane_blocked_until_` implement anti-flapping
(discourage immediately reversing a just-made change) so MOBIL doesn't
oscillate a vehicle back and forth between two lanes that both look equally
attractive tick to tick. `add_zone_wait`/`is_forced`/`grant_forced_clearance`
exist because junction yielding, left uncontested, can starve one traffic
stream forever if the other stream never has a gap — the design here is
reactive, not preventive: a vehicle's impatience builds *while it waits*,
and once granted a forced-clearance window it's allowed to push through
gaps it would normally refuse. That's the concrete mechanism behind the
"gridlocks are handled reactively... rather than prevented proactively"
limitation. `rescue_to` is the same idea taken further: an outright deadlock
gets broken by teleporting a stuck vehicle forward along its own route,
refusing only if the destination slot is actually occupied.

**Usage:** All of these are called from `TrafficSimulator`'s tick loop and
`JunctionArbiter` (both in `learn/07-simulation-core.md`), which is exactly
why the surrounding source comment frames this whole block as "Hooks used by
TrafficSimulator" — `Vehicle` is deliberately a passive object here, driven
by simulator-computed inputs (`lead`, `gap`, lane decisions) rather than
deciding globally-scoped things on its own.

**Check yourself:**
- MOBIL's incentive term needs to compare "how much better for me" against "how much worse for the vehicle behind me, weighted by politeness." Given that `Vehicle` only exposes `lane_politeness()` and not that full comparison, where must the rest of that computation live?
- Why cap `zone_wait_` at `kMaxZoneWait` (15 seconds) instead of letting it grow unbounded the longer a vehicle waits?
- What's the difference in intent between `is_forced()`/`grant_forced_clearance()` (junction impatience) and `rescue_to()` (deadlock rescue) — why does this codebase need both?

---

### Lines 187–221: private state

```cpp
private:
    static constexpr double kMaxZoneWait = 15.0;
    std::vector<Intersection*> run_router(const TrafficGraph& graph,
                                          const std::string& start_id,
                                          const std::string& target_id) const;

    Intersection* destination_;
    double max_speed_;
    double accel_;
    double max_accel_;
    double max_decel_;
    std::vector<Road*> route_;
    int route_index_;
    std::vector<std::string> travel_history_;
    bool is_reached_destination_;

    bool needs_reroute_ = false;
    bool route_dirty_ = false;
    double recompute_cooldown_ = 0.0;
    int recalculations_ = 0;
    std::string algorithm_ = "dijkstra";
    std::unique_ptr<PathFinding> router_;
    double travel_time_ = 0.0;

    double lane_change_cooldown_ = 0.0;
    int prev_lane_ = -1;
    double prev_lane_blocked_until_ = 0.0;

    double zone_wait_ = 0.0;
    double forced_until_ = -1.0;
    State state_ = State::Planning;
```

**Meaning:** Everything the public interface above was an accessor or
mutator for. `run_router` is a private helper — the comment above it says
why it exists: "so `request_route()` and `recompute_route()` can never
disagree about which one is in use." Note `algorithm_` defaults to
`"dijkstra"` and `router_` is a `unique_ptr<PathFinding>` — the Strategy
object itself, owned exclusively by this vehicle. `route_index_` defaults
implicitly (set in the constructor init list to `-1`) meaning "not on a
route yet." `prev_lane_` defaults to `-1` meaning "no lane just vacated."
`state_` defaults to `State::Planning`.

**Why:** grouping private state by feature (routing fields, then
reroute/cooldown flags, then lane-change fields, then junction-yielding
fields) rather than alphabetically or by declaration order in the `.cpp`
makes the header double as documentation of the vehicle's subsystems — you
can tell at a glance which fields belong to "routing" versus "lane
changing" versus "gridlock relief" just from the blank-line groupings and
the comments beside each field.

**Usage:** These are only ever touched from within `Vehicle.cpp` (or by
`VehicleTypes.hpp`'s subclasses, which only touch the *protected/public*
surface, not these `private` fields directly — the subclasses don't need
to, since none of them override movement or routing logic, only the hook
methods from lines 54–67).

**Check yourself:**
- Why is `run_router` private when `request_route`/`recompute_route` (its only two callers) are both public?
- `route_index_` is declared as a plain `int`, not `std::size_t`, even though it indexes into `route_` (a `vector`). Given that it needs to represent "not yet on a route" as `-1`, why does that matter?

---

## `cpp_backend/src/vehicles/Vehicle.cpp`

### Lines 16–38: IDM tuning constants

```cpp
constexpr double kIdmT = 0.05;
constexpr double kIdmS0 = 0.25;
constexpr double kIdmA = 2.5;
constexpr double kIdmB = 3.0;
```

**Meaning:** These four constants are the classic IDM parameters: `kIdmT` is
the desired time headway in seconds (how many seconds of following distance
a driver wants at their current speed), `kIdmS0` is the minimum bumper-to-
bumper gap at a standstill, `kIdmA` is comfortable acceleration, `kIdmB` is
comfortable (voluntary) deceleration.

**Why:** the comment above them is unusually candid about the tuning
history: "Tuned tighter than a textbook value on purpose... Retuned twice
now (1.0/0.25 -> 0.5/0.15 -> this) after screenshots showed the previous
pass still reading as too loose at a glance." This is a project that tunes
its physics constants by *visual inspection of the rendered output*, not
purely by matching a published IDM parameterization — worth knowing when
you're debugging "why does traffic look too tight/loose," because the
answer might legitimately be "someone eyeballed a screenshot," not a
mathematical error.

**Usage:** These feed directly into `idm_accel()` below — they're in an
anonymous namespace, so they're only visible within this translation unit,
not exported through the header. That means no other `.cpp` file can read
or override them; tuning IDM tightness is a change confined to this one
file.

**Check yourself:**
- Why does this codebase choose `kIdmT = 0.05`s — a tiny headway — instead of a textbook value closer to 1.5s? What kind of traffic is this meant to simulate?
- Why put these constants in an anonymous namespace instead of as `static constexpr` members on the `Vehicle` class?

---

### Lines 27–39: "close-up tuck-in" and contact/light constants

```cpp
constexpr double kCloseUpHorizon = 20.0;
constexpr double kCloseUpMargin = 1.5;
constexpr double kCloseUpBoost = 1.15;
constexpr double kCloseUpMinLeadSpeed = 2.0;
constexpr double kContactMargin = 0.15;
constexpr double kLightLookahead = 80.0;
constexpr double kAmberComfortDecel = 3.0;
constexpr double kLateralSpeed = 2.0;
```

**Meaning:** The comment above this block explains the problem it solves:
"Textbook IDM only ever REACTS to a gap that is already tight: once a
follower matches its leader's speed with room to spare, both terms of the
acceleration go to zero and the gap freezes at whatever spawn timing
happened to produce." `kCloseUpHorizon`/`kCloseUpMargin`/`kCloseUpBoost`/
`kCloseUpMinLeadSpeed` bound a special case where a vehicle that has matched
its leader's speed but still has slack in the gap will keep tucking in.
`kContactMargin` is the hard minimum bumper clearance kept everywhere.
`kLightLookahead` is how far before a stop line the vehicle starts
evaluating the signal. `kAmberComfortDecel` is the braking-comfort threshold
used to decide whether a vehicle can still stop comfortably for an amber
light. `kLateralSpeed` is the sideways drift speed for animating a lane
change (in m/s, feeding the `lateral_offset` easing seen in `advance()`).

**Why:** this is the codebase acknowledging a real, known gap in textbook
IDM and patching around it deliberately, rather than accepting looser
following in dense urban settings. It's the same design impulse as the
"tuned tighter than textbook" comment above — this project is explicitly
targeting a dense-urban look, and is willing to deviate from the reference
model to get it.

**Usage:** `kCloseUpHorizon`/`Margin`/`Boost`/`MinLeadSpeed` are consumed
inside `idm_accel()`. `kContactMargin` shows up in three separate places:
`entry_slot_free()`'s clearance math, the "never through the car in front"
clamp in `advance()`, and (indirectly) the stop-line calculation.
`kLightLookahead`/`kAmberComfortDecel` are used in the traffic-light branch
of `advance()`. `kLateralSpeed` is used in step 5 of `advance()` (the
lane-change drift easing).

**Check yourself:**
- Why does `kCloseUpMinLeadSpeed` exist — what's the danger of applying the "tuck in" boost when the leader is very slow or stopped?
- `kContactMargin` (0.15m) is explicitly noted as staying "under `kIdmS0`" (0.25m) — why would it be a bug if `kContactMargin` were *larger* than `kIdmS0`?

---

### Lines 41–72: multi-lane constants and `entry_slot_free`

```cpp
constexpr double kLaneWidth = 3.5;
constexpr double kLaneChangeCooldown = 5.0;
constexpr double kLaneReturnInhibit = 12.0;
constexpr double kStuckSpeed = 0.3;

bool entry_slot_free(const Road& road, int lane, double entry, double len) {
    for (const Vehicle* o : road.vehicles) {
        if (!o || o->lane != lane) continue;
        const double d = o->position_on_road - entry;
        const double needed = 0.5 * (o->length_meters() + len) + kContactMargin;
        if (std::abs(d) < needed) return false;
    }
    return true;
}
```

**Meaning:** `kLaneWidth` (3.5m) is used purely to compute how far
`lateral_offset` needs to visually jump when a lane change starts —
confirming again that lane width only matters for animation, not physics.
`kLaneChangeCooldown` (5s) and `kLaneReturnInhibit` (12s) throttle lane
changing. `kStuckSpeed` (0.3 m/s) is the threshold below which a vehicle
counts as "stopped" for stuck-time tracking. `entry_slot_free` loops every
vehicle `o` already on `road`, skips any not in the target `lane` (that
`o->lane != lane` comparison is, again, the discrete-lane check — there's
no continuous-position overlap test here, just "same integer lane or not"),
computes the along-road distance `d` from `entry`, and requires it to be at
least half the sum of both vehicles' lengths plus a fixed contact margin.

**Why:** the two comment blocks above this function are worth reading as
one unit. The first explains *why the check is needed at all*: the
simulator picks every leader once per tick before anything moves, so two
vehicles approaching a junction from different roads can both see an empty
road ahead and both try to hand over onto it in the same tick — this
function is what stops the second one. The second comment documents a real,
previously-shipped bug: this check used to require only *one* vehicle's
full length (picked by which side of `entry` it was on), which is only
correct when both vehicles happen to be the same length, and under-counts
clearance otherwise — "confirmed live as a car and a bus landing 0.75m into
each other at a handover." That's a direct, named consequence of the
project's per-vehicle-type `length_meters()` (a 6.5m bus vs. a 4.5m car)
combined with a symmetric clearance rule that used to be asymmetric.

**Usage:** Called from `advance()`'s road-handover step (step 6) and from
`rescue_to()` before teleporting a vehicle to a new road. It's also
explicitly cross-referenced to `JunctionArbiter.cpp`'s `gap_between()`,
which the comment says "requires half of BOTH vehicles' lengths for exactly
that reason" — i.e. this fix was applied consistently in both places once
the bug was found.

**Check yourself:**
- Why does `entry_slot_free` compare `o->lane != lane` before doing any distance math, instead of just checking distance across all lanes?
- Walk through what would happen at a car/bus handover if `needed` only used `len` (the arriving vehicle's length) and ignored `o->length_meters()` — which direction would the error go, and by how much?

---

### Lines 74–101: router construction and the Strategy-pattern glue

```cpp
std::unique_ptr<PathFinding> make_router(const std::string& name) {
    if (name == "astar") return std::make_unique<AStar>();
    if (name == "bfs")   return std::make_unique<BFS>();
    return std::make_unique<Dijkstra>();
}
```
```cpp
std::vector<Intersection*> Vehicle::run_router(const TrafficGraph& graph,
                                               const std::string& start_id,
                                               const std::string& target_id) const {
    return router_->find_path(graph, start_id, target_id);
}

void Vehicle::set_algorithm(const std::string& name) {
    if (name == algorithm_ && router_) return;
    algorithm_ = name;
    router_ = make_router(name);
}

void Vehicle::set_cost_mode(CostMode mode) {
    if (router_) router_->set_cost_mode(mode);
}
```

**Meaning:** `make_router` is a free function in the anonymous namespace,
noted by the comment above it as "the only place in the codebase that names
Dijkstra, AStar or BFS directly" — it takes a name string and returns the
matching concrete `PathFinding` subclass, defaulting to `Dijkstra` for
anything unrecognized. `run_router` calls through the stored `router_`
pointer via the base-class virtual `find_path`. `set_algorithm` is a
no-op guard (skip rebuilding if the name hasn't changed and a router
already exists) followed by swapping in a freshly constructed strategy
object. `set_cost_mode` forwards to the strategy if one exists.

**Why:** this is the textbook Strategy pattern, and the comment's claim —
"the only place... that names Dijkstra, AStar or BFS directly" — is the
whole point of the pattern: every other piece of code that wants a route
just calls `run_router`/`find_path` through the `PathFinding` base-class
interface, never knowing or caring which concrete algorithm is behind it.
Concentrating construction in one function means adding a fifth algorithm
later is a one-function change, not a search-and-replace across the
codebase.

**Usage:** `run_router` is called by both `request_route()` and
`recompute_route()`, which is exactly the "can never disagree about which
[strategy] is in use" guarantee promised in the header comment — see
`learn/04-routing.md` for `PathFinding`, `Dijkstra`, `AStar`, and `BFS`
themselves.

**Check yourself:**
- Why does `set_algorithm` check `router_` for null in its guard condition, given that the constructor always initializes `router_`?
- If you added a new `PathFinding` subclass `Contraction`, which single function in this file would you edit to make it selectable by name?

---

### Lines 103–130: the constructor body

```cpp
Vehicle::Vehicle(std::string id, double max_speed, double max_accel, double max_decel)
    : id(id), current_road(nullptr), position_on_road(0.0), current_speed(0.0),
      destination_(nullptr), max_speed_(max_speed), accel_(0.0), max_accel_(max_accel),
      max_decel_(max_decel), route_index_(-1), is_reached_destination_(false),
      router_(make_router(algorithm_)) {

    unsigned h = 0;
    for (char c : id) h = h * 31u + static_cast<unsigned>(c);
    idm_factor = 0.8 + 0.2 * ((h % 100u) / 99.0);
    ...
}
```

**Meaning:** The member initializer list sets every field to a clean
starting value: no road, position/speed zero, no destination, the passed-in
speed limits, `route_index_ = -1` ("not on a route"), and constructs
`router_` immediately via `make_router(algorithm_)` (using the default
`"dijkstra"` from the header). Inside the body, `h` is a simple polynomial
rolling hash (multiply-by-31-and-add, the classic Java-`String.hashCode`-style
hash) computed over every character of `id`. `idm_factor` is then derived
from `h % 100` scaled into `[0.8, 1.0]`.

**Why:** the comment explains the intent directly — "Derive the speed
personality from a hash of the id, so identical cars do not drive
identically and a run stays reproducible." A hash of the id is
deterministic (same id always produces the same `idm_factor`), which is
what "reproducible" means here: re-running the same scenario with the same
spawned ids gives identical driver personalities every time, unlike seeding
from a random-number generator's current state, which would depend on call
order.

**Usage:** `idm_factor` is read every tick in `desired_speed_on()`, so this
one hash computed once at construction shapes this vehicle's behavior for
its entire lifetime.

**Check yourself:**
- Why use a hash of the vehicle's `id` for `idm_factor` instead of a `std::mt19937` seeded once at simulator startup?
- What does initializing `router_` via `make_router(algorithm_)` in the initializer list (rather than leaving it null and lazily constructing later) buy you?

---

### Lines 115–129: reconstructing `num_id` from the id string

```cpp
std::size_t digits_start = id.size();
while (digits_start > 0 && std::isdigit(static_cast<unsigned char>(id[digits_start - 1]))) {
    --digits_start;
}
num_id = digits_start < id.size()
             ? static_cast<uint32_t>(std::strtoul(id.c_str() + digits_start, nullptr, 10))
             : static_cast<uint32_t>(h);
```

**Meaning:** This walks backward from the end of `id`, decrementing
`digits_start` past every trailing digit character (`std::isdigit`, cast to
`unsigned char` first — the standard defensive move against passing a
negative `char` value into `isdigit`, which is undefined behavior for
values outside `unsigned char`/`EOF`). Once the loop stops, `digits_start`
points at the first digit of the trailing numeric run (e.g. `"car42"` →
`digits_start` at index 3, `"42"`). If a digit run was found
(`digits_start < id.size()`), `num_id` is parsed from that substring with
`strtoul`; otherwise (no trailing digits at all) it falls back to the hash
`h` computed just above.

**Why:** the comment is explicit that this replaced a *broken* check: "this
reconstructs correctly regardless of which vehicle type prefixes it, unlike
a fixed `'v'` prefix check (which never matched: real prefixes are
car/bus/emergency/motorbike)." In other words, an earlier version of this
code assumed every id started with `'v'`, which was simply wrong for this
codebase's actual naming convention. The comment also nails down *why this
has to keep working*: "The frontend rebuilds the display id as
`TYPE_NAMES[type] + num_id` from the binary snapshot, so this has to stay
reversible." That's a strong invariant — if `VehicleFactory`/`spawn_locked`'s
id format (`"<type><counter>"`) ever changed, this parsing logic and the
frontend's reconstruction logic would both need to change together, or ids
would silently render wrong on screen despite the backend working fine.

**Usage:** `num_id` is what gets streamed in the binary snapshot format
(cheaper than the full string `id`), and the frontend reconstitutes a
human-readable id purely from `type` + `num_id` — never from the string
`id` itself over the wire.

**Check yourself:**
- Why cast the character to `unsigned char` before calling `std::isdigit`? What could go wrong without the cast?
- If a caller constructed a `Vehicle` with a hand-built id like `"testcar"` (no trailing digits), what would `num_id` end up being, and would it be stable across runs?

---

### Lines 132–163: `idm_accel` — the car-following core

```cpp
double Vehicle::idm_accel(double v, double v0, double v_lead, double gap) {
    v0 = std::max(0.5, v0);
    gap = std::max(0.1, gap);
    const double delta_v = v - v_lead;
    const double sqrt_ab = 2.0 * std::sqrt(kIdmA * kIdmB);
    const double s_star = kIdmS0 + std::max(0.0, v * kIdmT + (v * delta_v) / sqrt_ab);
```

**Meaning:** `v0` (desired speed) is floored at `0.5` m/s and `gap` at
`0.1` m — both guard against division-by-zero or degenerate math further
down. `delta_v` is the closing speed relative to the leader (positive means
approaching). `sqrt_ab` is `2*sqrt(a*b)`, a fixed normalizing term from the
IDM formula. `s_star` — the "desired dynamic gap" — is the minimum
standstill gap `kIdmS0` plus a speed-and-closing-rate-dependent term,
clamped to never go negative (a vehicle moving away from its leader
shouldn't get a *smaller* desired gap than standing still would demand).

**Why:** this is the textbook IDM `s*` formula, verbatim — the standard
model for "how much gap does a driver want, given how fast I'm going and
how fast I'm closing on the car ahead."

**Usage:** `s_star` is used both in the free-flow-boost check below and in
the final acceleration formula's braking term.

**Check yourself:**
- Why clamp `v0` and `gap` at the top of this function instead of trusting callers to never pass zero or negative values?

---

```cpp
    double v0_eff = v0;
    if (v_lead > kCloseUpMinLeadSpeed &&
        gap < kCloseUpHorizon && gap > s_star * kCloseUpMargin) {
        v0_eff = v0 * kCloseUpBoost;
    }
    double a = kIdmA * (1.0 - std::pow(v / v0_eff, 4) - std::pow(s_star / gap, 2));

    if (v < 3.0 && gap > 2.0 * s_star && v_lead >= v) {
        a = std::max(a, kIdmA * 0.95);
    }
    return a;
}
```

**Meaning:** `v0_eff` starts equal to `v0`, then gets boosted by
`kCloseUpBoost` (1.15×) only if the leader is genuinely moving
(`v_lead > kCloseUpMinLeadSpeed`), within sight (`gap < kCloseUpHorizon`),
and there's slack worth taking up (`gap > s_star * kCloseUpMargin`) — this
is the "dense-urban tuck-in" behavior described earlier. The main IDM
acceleration formula, `a = kIdmA * (1 - (v/v0_eff)^4 - (s*/gap)^2)`, has two
competing terms: the free-flow term pulls acceleration toward zero as `v`
approaches `v0_eff` (only the ceiling moves when boosted, per the comment),
and the interaction term pushes acceleration sharply negative as `gap`
shrinks below `s_star` — this is what gives IDM its collision-avoidance
property. Finally, a "crisp launch" override: if the vehicle is slow
(`v < 3.0`), has plenty of room (`gap > 2*s_star`), and the leader is at
least as fast (`v_lead >= v`), the acceleration is floored at 95% of max —
avoiding a sluggish crawl away from a stop when there's clearly no reason
to hold back.

**Why:** the comment on the `v_lead` guard is the important safety note
here: "The `v_lead` guard is what keeps this from being dangerous:
`advance()` calls in here with `v_lead = 0` and a VIRTUAL gap to the stop
line when braking for a red light, and a genuinely stopped leader looks the
same. Chasing either would accelerate a vehicle at the thing it must stop
for." In other words, without `kCloseUpMinLeadSpeed`, this "boost" logic
could accidentally interpret "the gap to a red light" as "slack worth
closing" and accelerate the vehicle toward the stop line it's supposed to
be braking for. Requiring the leader to actually be moving faster than
2 m/s rules that out, since a virtual stop-line "leader" is always encoded
with `v_lead = 0`.

**Usage:** This is called once per tick, per vehicle, from `advance()`,
with `gap` set either to the real distance to a leading vehicle or to a
synthetic distance to a red light's stop line.

**Check yourself:**
- Walk through what `idm_accel` returns if `gap` is very small relative to `s_star` (a near-collision). Which term dominates, and does the sign make sense?
- Why does the "crisp launch" override use `std::max(a, ...)` rather than just overwriting `a`?

---

### Lines 165–169: `desired_speed_on`

```cpp
double Vehicle::desired_speed_on(const Road& road) const {
    const double free_flow =
        std::min(max_speed_, road.effective_speed_ms(congestion_sensitivity()));
    return free_flow * speed_factor() * idm_factor;
}
```

**Meaning:** `free_flow` takes the smaller of this vehicle's own hard speed
cap (`max_speed_`) and the road's *effective* speed — `Road::effective_speed_ms`
takes a congestion-sensitivity argument, meaning congestion degrades the
usable speed differently per vehicle type. The final desired speed
multiplies that free-flow figure by the vehicle type's `speed_factor()`
hook and this specific vehicle's `idm_factor` personality.

**Why:** this is the concrete formula referenced earlier — three
multiplicative layers (road/congestion, vehicle type, individual
personality) each contribute independently, so an `EmergencyVehicle` with
`speed_factor() == 1.35` and low `congestion_sensitivity()` (0.25, from
`VehicleTypes.hpp`) barely slows down in traffic that would meaningfully
slow a `Car`.

**Usage:** This is the `v0` argument passed to `idm_accel()` in `advance()`
— it's recomputed fresh every tick since road congestion changes over time.

**Check yourself:**
- If a road's speed limit is 20 m/s but this vehicle's `max_speed_` is 16.7 m/s, which one wins in `free_flow`, and why does that make sense physically?

---

### Lines 171–245: `request_route` and `recompute_route`

```cpp
void Vehicle::request_route(TrafficGraph* graph, Intersection* start, Intersection* target) {
    if (!graph || !start || !target) return;
    destination_ = target;
    is_reached_destination_ = false;
    std::vector<Intersection*> node_path = run_router(*graph, start->name, target->name);
    route_.clear();
    for (size_t i = 0; i + 1 < node_path.size(); ++i) {
        Intersection* u = node_path[i];
        Intersection* v = node_path[i + 1];
        Road* found_road = nullptr;
        for (Road* road : u->outgoing_edges) {
            if (road->target == v) { found_road = road; break; }
        }
        if (found_road) route_.push_back(found_road);
    }
    if (!route_.empty()) {
        route_index_ = 0;
        current_road = route_[route_index_];
        position_on_road = 0.0;
        current_road->add_vehicle(this);
    } else {
        route_index_ = -1;
        current_road = nullptr;
    }
}
```

**Meaning:** After a null-argument guard, this stores the target and clears
the "arrived" flag, then calls `run_router` to get a path as a sequence of
`Intersection*` nodes. It then converts that *node* path into a *road* path:
for each consecutive pair `(u, v)` of intersections, it linearly scans `u`'s
`outgoing_edges` for the `Road` whose `target` is `v`. If a route was found,
the vehicle is placed at the very start of the first road (`position_on_road = 0.0`)
and registered onto it via `current_road->add_vehicle(this)`. If nothing was
found (disconnected graph, or `node_path` was empty/degenerate),
`route_index_` is reset to `-1` and `current_road` to `nullptr`.

**Why:** the pathfinding strategies (`learn/04-routing.md`) operate over
*intersections* (graph nodes) because that's the natural unit for
Dijkstra/A*/BFS, but a `Vehicle` needs to drive along *roads* (graph edges),
so this conversion step is necessary glue between the two representations.
The inner linear scan over `outgoing_edges` is a simple O(degree) lookup —
fine given intersections in a road network have a small, bounded number of
outgoing roads, not a scalability concern here.

**Usage:** `request_route` is the first call made on a freshly-spawned
vehicle (see `TrafficSimulator::spawn_locked` in `learn/07-simulation-core.md`),
and it's what actually attaches the vehicle to a `Road`'s vehicle list for
the first time via `add_vehicle`.

**Check yourself:**
- What happens to a vehicle whose `node_path` comes back non-empty, but for which the inner loop never finds a matching `Road` for any consecutive pair (i.e. `route_` ends up empty)? Trace through the `if (!route_.empty())` branch.
- Why does this function need to search `outgoing_edges` at all — why can't the node path from the router already just *be* a road path?

---

```cpp
void Vehicle::recompute_route(TrafficGraph* graph, Intersection* target) {
    if (!graph || !target) return;
    destination_ = target;
    std::string start_node_id = current_road ? current_road->target->name : "";
    if (start_node_id.empty()) return;
    std::vector<Intersection*> node_path = run_router(*graph, start_node_id, target->name);
    ...
    if (!new_route.empty()) {
        Road* saved_current = current_road;
        route_.clear();
        route_.push_back(saved_current);
        route_.insert(route_.end(), new_route.begin(), new_route.end());
        route_index_ = 0;
        is_reached_destination_ = false;
        route_dirty_ = false;
        recompute_cooldown_ = 5.0;
        ++recalculations_;
    }
}
```

**Meaning:** Unlike `request_route`, this doesn't start from an
`Intersection` — it starts from wherever the vehicle *currently is*: the
downstream end of `current_road` (`current_road->target`, the intersection
this road leads into). It reroutes from there to `target`, builds the new
road-path the same way as before, and — if successful — **prepends the
vehicle's current road** onto the new route before installing it, resetting
`route_index_` back to `0`. It also clears `is_reached_destination_` and
`route_dirty_`, sets a 5-second `recompute_cooldown_`, and increments the
`recalculations_` counter.

**Why:** rerouting mid-trip has to preserve the vehicle's current physical
position — it can't just be assigned a route starting at the *next*
intersection, because it's still physically driving along `current_road`.
Prepending `saved_current` and setting `route_index_ = 0` keeps
`current_road` consistent with `route_[route_index_]`, which every other
part of `Vehicle` and `advance()` assumes holds. The 5-second cooldown
(`recompute_cooldown_ = 5.0`, gated by `can_recompute()` in the header) is
what stops a vehicle from thrashing — recomputing every single tick if,
say, congestion notifications keep firing.

**Usage:** Called by the simulator's tick loop whenever `consume_reroute_flag()`
returns true and `can_recompute()` allows it — see `learn/06-events-observer.md`
for what triggers `needs_reroute_` via `onNotify`, and `learn/07-simulation-core.md`
for where in the tick loop this gets invoked.

**Check yourself:**
- Why must `route_` be rebuilt with `saved_current` prepended, rather than just replacing `route_` with `new_route` directly?
- What's the purpose of `recompute_cooldown_`, and what field elsewhere in the class reads it before allowing another call?

---

### Lines 247–312: getters, `remaining_route_contains`, `onNotify`, `consume_reroute_flag`

```cpp
bool Vehicle::remaining_route_contains(const std::string& road_id) const {
    for (std::size_t i = static_cast<std::size_t>(std::max(0, route_index_)); i < route_.size(); ++i) {
        if (route_[i] && route_[i]->id == road_id) return true;
    }
    return false;
}
```

**Meaning:** Iterates `route_` starting at `route_index_` (clamped to `0`
first, via `std::max(0, route_index_)`, in case `route_index_` is the
sentinel `-1`) through the end of the vector, and returns true if any
remaining road matches `road_id`.

**Why:** the `std::max(0, route_index_)` clamp exists because `route_index_`
can legitimately be `-1` (no route yet), and casting `-1` directly to
`std::size_t` would wrap around to a huge unsigned value, turning the loop
into either an immediate no-op that silently skips valid indices or (worse)
undefined-adjacent iteration depending on `route_.size()` — clamping first
avoids that entirely.

**Usage:** This is the "remaining" qualifier promised in the header's
comment — used by `onNotify` (next) to check whether an event on some road
actually matters to *this* vehicle's future driving, ignoring roads it
already passed.

**Check yourself:**
- What would go wrong if this function used `route_index_` directly (cast to `std::size_t`) without the `std::max(0, ...)` clamp?

---

```cpp
void Vehicle::onNotify(const TrafficNotification& notification) {
    using Kind = TrafficNotification::Kind;
    switch (notification.kind) {
        case Kind::RoadClosed:
        case Kind::Accident:
        case Kind::Congestion:
            if (notification.roadId.empty() || remaining_route_contains(notification.roadId)) {
                needs_reroute_ = true;
            }
            return;
        case Kind::RoadReopened:
        case Kind::AccidentCleared:
        case Kind::CongestionCleared:
            if (!notification.roadId.empty() && !remaining_route_contains(notification.roadId)) {
                needs_reroute_ = true;
                const std::size_t h = std::hash<std::string>{}(id);
                const double stagger = 1.0 + static_cast<double>(h % 100) / 10.0;
                recompute_cooldown_ = std::max(recompute_cooldown_, stagger);
            }
            return;
        case Kind::TrafficLight:
        case Kind::TrafficLightRestored:
            return;
    }
}
```

**Meaning:** A `switch` over the notification kind, grouped into three
cases. "Bad news" kinds (road closed, accident, congestion) flag a reroute
if either the notification is network-wide (`roadId` empty) or the affected
road is still ahead of this vehicle. "Good news" kinds (reopened, cleared)
flag a reroute only if the *now-better* road is **not** already on the
route — i.e. "a shortcut just became available that I'm not using," and
additionally staggers the recompute cooldown using a per-vehicle hash so
not every vehicle recomputes in the same tick. Traffic-light kinds are
explicitly ignored here — `return` with no action.

**Why:** the asymmetry between the two reroute branches is deliberate and
worth noticing: for bad news, *any* vehicle already planning to use the
affected road should reroute. For good news, a vehicle should only reroute
if it *isn't* already benefiting — rerouting a vehicle that's already using
the now-reopened road would be pointless churn. The staggering logic (a
hash-derived jitter added to `recompute_cooldown_`) exists to avoid a
thundering-herd effect: if a major road reopens and hundreds of vehicles
were all waiting on it, without staggering they'd all call
`recompute_route()` on the very next eligible tick simultaneously, which is
an avoidable computational spike. `TrafficLight`/`TrafficLightRestored` are
ignored because a signal changing color doesn't invalidate a route the way
a closure does — the vehicle just waits or goes, which `advance()` already
handles.

**Usage:** This is the `IObserver` override invoked by `TrafficController`
(`learn/06-events-observer.md`) whenever a `TrafficNotification` fires
on the network. It only ever sets `needs_reroute_`, consistent with the
"defer the real work" design already discussed for this method.

**Check yourself:**
- Why does the "cleared" branch check `!remaining_route_contains(...)` (reroute if NOT already using it) while the "closed" branch checks the opposite polarity?
- What concrete problem does the `stagger` computation prevent, and why derive it from a hash of `id` rather than, say, `rand()`?

---

```cpp
bool Vehicle::consume_reroute_flag() {
    const bool flagged = needs_reroute_;
    if (flagged) needs_reroute_ = false;
    return flagged;
}
```

**Meaning:** Reads `needs_reroute_` into a local, clears it if it was set,
and returns the value read.

**Why:** this "read, clear, return" pattern is the standard way to implement
a one-shot flag that's safe to poll every tick without double-processing —
the caller gets a `true` exactly once per `onNotify`-triggered reroute
request, not once per tick for as long as the flag happens to stay set.

**Usage:** Called by the simulator's tick loop; a `true` return leads to a
`recompute_route()` call (subject to `can_recompute()`'s cooldown gate).

**Check yourself:**
- What would happen if `consume_reroute_flag()` didn't clear `needs_reroute_`, and the simulator called it once per tick?

---

### Lines 315–331: `lane_change_ready` and `set_lane`

```cpp
bool Vehicle::lane_change_ready(double /*sim_time*/) const {
    return current_road && lane_change_cooldown_ <= 0.0 &&
           std::abs(lateral_offset - target_lateral_offset) < 0.05;
}

void Vehicle::set_lane(int new_lane, double sim_time) {
    if (new_lane == lane) return;
    prev_lane_ = lane;
    prev_lane_blocked_until_ = sim_time + kLaneReturnInhibit;
    lateral_offset += (lane - new_lane) * kLaneWidth;
    lane = new_lane;
    target_lateral_offset = 0.0;
    lane_change_cooldown_ = kLaneChangeCooldown;
}
```

**Meaning:** `lane_change_ready` (note `sim_time` is unused — commented out
as `/*sim_time*/` to silence unused-parameter warnings while keeping the
signature stable for callers) returns true only if the vehicle is on a road
at all, its cooldown has fully elapsed, and it isn't still visually
mid-drift from the last change. `set_lane` is a no-op if `new_lane` equals
the current `lane`; otherwise it records the vacated lane and a
return-inhibit deadline, then does the "animate across" trick described
earlier: it *adds* `(lane - new_lane) * kLaneWidth` to the current
`lateral_offset` (which pushes the drawn position to roughly where the old
lane's centre was, relative to the new lane), flips `lane` to the new
value, resets `target_lateral_offset` to `0.0` (the new lane's centre, the
target for the drift-easing in `advance()`), and resets the cooldown timer.

**Why:** this confirms precisely what was flagged in the header: `set_lane`
changes the vehicle's *actual, discrete* lane assignment instantly and
unconditionally (once called) — the "smooth slide" is purely a rendering
illusion layered on afterward via `lateral_offset` easing toward
`target_lateral_offset` in step 5 of `advance()`. There is no tick where
the vehicle is *physically* half in one lane and half in another for
purposes of gap-finding or collision checks; `entry_slot_free` and every
other spatial check reads the already-updated integer `lane` the instant
`set_lane` returns. `prev_lane_`/`prev_lane_blocked_until_` implement the
anti-flapping behavior discussed earlier — a vehicle that just changed
lanes is discouraged (not forbidden — "inhibit," not "block") from
immediately changing back for `kLaneReturnInhibit` (12) seconds.

**Usage:** Called by the simulator's MOBIL decision logic once it decides a
lane change is both incentivized and safe, only after confirming
`lane_change_ready()`.

**Check yourself:**
- At the instant `set_lane()` returns, is the vehicle safe from a collision-detection standpoint in its new lane, or does that depend entirely on the caller having already verified the target lane was clear? (Look at what `set_lane` itself does and doesn't check.)
- Why is `prev_lane_blocked_until_` framed as an "inhibit" rather than an outright block — what does the header's `prev_lane_blocked_until()` accessor let a caller decide?

---

### Lines 333–387: `transfer_to_road` and `rescue_to`

```cpp
void Vehicle::transfer_to_road(Road* new_road, double progress_meters) {
    if (!new_road) return;
    current_road = new_road;
    position_on_road = std::clamp(progress_meters, 0.0, new_road->length);
    const std::vector<int> ok = allowed_lanes(new_road->get_lanes());
    if (std::find(ok.begin(), ok.end(), lane) == ok.end()) {
        lane = ok.empty() ? 0 : ok.front();
        target_lateral_offset = 0.0;
    }
    route_.clear();
    route_.push_back(new_road);
    route_index_ = 0;
    needs_reroute_ = true;
    new_road->add_vehicle(this);
}
```

**Meaning:** After a null guard, this directly reassigns `current_road`
(the comment above notes the old road is *already gone* — the caller must
detach the vehicle from it first, since calling `remove_vehicle` on a
deleted `Road` is a use-after-free) and clamps `position_on_road` into the
new road's valid range. It then checks whether the vehicle's current `lane`
is still legal on the new road via `allowed_lanes()` (a `VehicleTypes.hpp`
hook — see below); if not, it falls back to the first allowed lane. Finally
it truncates the route down to just this one road, marks `route_index_ = 0`,
sets `needs_reroute_ = true` so the simulator replans the rest next tick,
and registers the vehicle onto the new road.

**Why:** this exists for *live map editing* — splitting a road in two while
the simulation is running, mid-vehicle. The design goal, per the comment,
is that "the driver does not notice the edit: same speed, same lane if that
lane still exists, same sideways drift" — this function preserves every bit
of motion state it can (speed and `lateral_offset` are untouched by this
function entirely) and only touches what genuinely has to change (which
`Road` object, and `lane` only if the new road has fewer lanes than
required).

**Usage:** Called by whatever live-editing code performs a road split
(`learn/08-supporting-systems.md` covers `MapEditor`) — it's the bridge
between "the graph topology just changed under a moving vehicle" and "the
vehicle keeps driving without a visible glitch."

**Check yourself:**
- Why does this function set `needs_reroute_ = true` directly instead of calling `recompute_route()` itself?
- What's the significance of the comment "the caller must have detached this vehicle from it first" — what would happen if that precondition were violated?

---

```cpp
bool Vehicle::rescue_to(std::size_t new_index) {
    if (new_index >= route_.size()) return false;
    Road* dest = route_[new_index];
    if (!dest) return false;
    int dest_lane = lane;
    const std::vector<int> ok = allowed_lanes(dest->get_lanes());
    if (std::find(ok.begin(), ok.end(), dest_lane) == ok.end()) {
        dest_lane = ok.empty() ? 0 : ok.front();
    }
    if (!entry_slot_free(*dest, dest_lane, 0.0, length_meters())) return false;
    if (current_road) current_road->remove_vehicle(this);
    route_index_ = static_cast<int>(new_index);
    current_road = dest;
    position_on_road = 0.0;
    lane = dest_lane;
    current_speed = 0.0;
    accel_ = 0.0;
    stuck_time = 0.0;
    zone_wait_ = 0.0;
    current_road->add_vehicle(this);
    return true;
}
```

**Meaning:** Bounds-checks `new_index` against the route length and rejects
a null destination road. Picks a valid lane on the destination road the same
way `transfer_to_road` does. Then — critically — calls `entry_slot_free`
before doing anything else, and bails out (`return false`) if the target
slot is occupied. Only if the slot is free does it detach from the current
road, jump the vehicle directly to position `0.0` on `dest`, zero its speed
and acceleration, reset `stuck_time` and `zone_wait_`, and attach to the new
road.

**Why:** this is the "last resort" deadlock breaker described in the
header. The comment there says it plainly: "Breaking the deadlock must not
mean materialising inside someone else. Leave the stuck counters alone so
the next tick tries again." That's why the `entry_slot_free` check comes
*before* any mutation — a failed rescue must be a true no-op (nothing
detached, nothing moved, `stuck_time`/`zone_wait_` untouched) so the caller
can safely retry it next tick without having already made things worse.
Returning `bool` rather than `void` lets the caller (the simulator, which
"counts rescues" per the header comment) distinguish an actual jump from a
refusal — important because "a refusal repeated every tick is not a
rescue," i.e. the caller shouldn't count a same-tick retry as two separate
rescue events.

**Usage:** Called by `TrafficSimulator`'s gridlock-detection logic
(`learn/07-simulation-core.md`) once a vehicle has been judged stuck for
long enough that a normal reroute won't help — it literally teleports the
vehicle to a later point on its own already-planned route, skipping
whatever's jammed in between.

**Check yourself:**
- Why is the `entry_slot_free` check placed before any state mutation, rather than, say, after moving the vehicle and then undoing the move if the slot turned out occupied?
- What real-world interpretation would you give to `rescue_to` succeeding — is this modeling anything a real driver does, or is it purely a simulation-integrity safety valve? (Compare to the honest framing of "gridlocks are handled reactively" in the project's known limitations.)

---

### Lines 390–528: `advance()` — the per-tick movement function

```cpp
void Vehicle::advance(double dt, const Vehicle* lead, double gap, double sim_time) {
    if (is_reached_destination_ || !current_road) return;
    travel_time_ += dt;
    lane_change_cooldown_ = std::max(0.0, lane_change_cooldown_ - dt);
    recompute_cooldown_ = std::max(0.0, recompute_cooldown_ - dt);
```

**Meaning:** An early return if the vehicle has already arrived or has no
current road (nothing to advance). Otherwise, `dt` (this tick's elapsed
simulated time) is added to `travel_time_`, and both cooldown timers are
decremented, floored at `0.0` so they never go negative.

**Why:** ticking cooldowns down here, unconditionally, at the top of the
one function guaranteed to run once per vehicle per tick, is simpler and
more reliable than trying to decrement them from multiple call sites.

**Usage:** `lane_change_cooldown_` gates `lane_change_ready()`;
`recompute_cooldown_` gates `can_recompute()`.

---

```cpp
    const double v0 = desired_speed_on(*current_road);
    double effective_gap = gap;
    Intersection* next_node = current_road->target;
    const bool forced = is_forced(sim_time);
    if (!ignores_lights() && !forced && next_node && next_node->is_signalized() &&
        current_road->signal_group >= 0) {
        const double dist_to_stop = current_road->stop_line_position() - position_on_road;
        if (dist_to_stop < kLightLookahead) {
            const unsigned int light = next_node->signal_state_for(current_road->signal_group, sim_time);
            bool must_stop = (light == TrafficLight::Red);
            if (light == TrafficLight::Amber) {
                const double d = std::max(0.5, dist_to_stop);
                must_stop = (current_speed * current_speed) / (2.0 * d) <= kAmberComfortDecel;
            }
            if (must_stop) {
                const double stop_gap = dist_to_stop - length_meters() * 0.5;
                if (stop_gap > -1.0) {
                    effective_gap = std::min(effective_gap, std::max(0.1, stop_gap));
                    lead = nullptr;
                }
            }
        }
    }
```

**Meaning:** `v0` is this tick's desired speed. `effective_gap` starts equal
to the simulator-supplied `gap` but may be shortened below. The light-check
guard fires only if this vehicle type doesn't ignore lights
(`ignores_lights()`), it isn't currently under a forced-clearance grant
(`is_forced`), the road actually leads into a signalized intersection, and
this road belongs to a real signal group. Given all that, it computes the
distance to the stop line and, if within lookahead range, reads the signal
color for this approach's specific group and time. Red always requires
stopping. Amber requires stopping only if the vehicle *could* comfortably
brake in time (using the standard `v²/(2d) <= decel` physics check against
`kAmberComfortDecel`) — otherwise it's already too close to brake
comfortably and is allowed through, exactly like a real driver facing a
dilemma zone. If a stop is required and the vehicle hasn't already passed
the line (`stop_gap > -1.0`), `effective_gap` is shortened to the (clamped)
distance to the line, and `lead` is nulled out — the stop line itself
becomes the "obstacle" IDM reacts to instead of whatever vehicle was ahead.

**Why:** the comment "the colour is the one shown to THIS approach's phase
group: the crossing street is showing the opposite colour at the same
moment" is a reminder that `TrafficLight` state is per-approach, not global
— see `learn/03-traffic-lights.md`. The amber-dilemma-zone logic is a
deliberate rejection of "treat amber as red," which the comment says
"is what puts a queue at every junction" — a vehicle already too close to
stop safely should be allowed to clear the intersection, matching real
driving behavior and avoiding an artificial jam right at the stop line.

**Usage:** This whole block is where `ignores_lights()` (an
`EmergencyVehicle` override) and `is_forced()` (junction-yielding relief)
both short-circuit the entire signal check — an emergency vehicle or one
currently granted forced clearance never enters this branch at all.

---

```cpp
    if (is_reached_destination_) {
        state_ = State::Arrived;
    } else if (route_.empty()) {
        state_ = State::Planning;
    } else if (effective_gap < gap) {
        state_ = State::WaitingAtLight;
    } else if (current_speed < 0.5 && effective_gap < 2.0 * length_meters()) {
        state_ = State::Blocked;
    } else {
        state_ = State::Moving;
    }
```

**Meaning:** This sets `state_` (the `State` enum from the header) based on
what was just computed, in a strict priority order the comment names
explicitly: a light check outranks everything else, because "a vehicle
stopped at a light is not blocked, it is obeying." `effective_gap < gap` is
precisely the signal that the light-check block above actually shortened
the gap — a cheap, correct way to detect "was this a light stop" without a
separate boolean. Otherwise, low speed with a tight real gap means
`Blocked` (likely gridlock-adjacent); anything else moving is `Moving`.

**Why:** computing `state_` from already-derived values (`effective_gap`
vs. `gap`) rather than re-deriving "was there a light" from scratch is
economical and, more importantly, guaranteed consistent with what the
physics step below is about to actually do — there's no way for `state_` to
disagree with the deceleration `idm_accel` is about to compute, since
they're driven by the same `effective_gap`.

**Usage:** This is the field read by `state()` and streamed to the
frontend, giving the honest, disambiguated "why" for `/api/state` that
motivated the `State` enum's existence in the first place.

**Check yourself:**
- Why does `effective_gap < gap` reliably indicate "the light shortened this" rather than, say, the real leader's gap also legitimately being smaller than the raw `gap` argument?
- Under what conditions could a vehicle be `Blocked` even though no traffic light is involved at all?

---

```cpp
    const double v_lead = lead ? lead->current_speed : 0.0;
    accel_ = idm_accel(current_speed, v0, v_lead, effective_gap);
    accel_ = std::clamp(accel_, -max_decel_, max_accel_);
    current_speed = std::max(0.0, current_speed + accel_ * dt);
```

**Meaning:** `v_lead` is the leader's speed, or `0.0` if `lead` is null
(covers both "no leader" and "stopped for a light," per the nulling-out
above). `idm_accel` is called with the four values built so far. The result
is clamped to this vehicle's own physical limits (`max_decel_`/`max_accel_`
— note IDM's raw output isn't trusted to respect a specific vehicle's
hardware limits on its own). `current_speed` is integrated forward by one
Euler step (`speed + accel * dt`), floored at `0.0` so a vehicle can't end
up with negative speed.

**Why:** clamping `accel_` after `idm_accel` returns is what lets
`max_accel_`/`max_decel_` (set per-vehicle in the constructor, distinct
from the *type's* `speed_factor()`) actually matter — otherwise IDM's
formula alone could, in principle, request an acceleration beyond what a
specific vehicle's tuning allows.

**Usage:** `accel_` is exposed via `get_acceleration()`; `current_speed` is
what everything downstream (the movement step, next tick's `desired_speed_on`
comparison, `stuck_time` tracking) reads.

---

```cpp
    double step = current_speed * dt;
    const double max_step = std::max(0.0, effective_gap - kContactMargin);
    if (step > max_step) {
        step = max_step;
        current_speed = dt > 1e-9 ? step / dt : 0.0;
    }
    position_on_road += step;
    last_gap = effective_gap;
```

**Meaning:** `step` is the naive distance this tick's speed would cover.
`max_step` caps that at the gap available minus the contact margin. If the
naive step would overshoot that cap, both `step` *and* `current_speed`
(recomputed backward from the clamped step) are corrected — this second
correction matters: without it, `current_speed` would report a speed the
vehicle didn't actually achieve, since it was clamped for *this tick's
movement* but the speed variable itself wouldn't reflect that clamp.
`position_on_road` advances by the (possibly clamped) `step`, and
`last_gap` records what gap was actually used, for diagnostics.

**Why:** the comment title in the source, "never through the car in
front," names the invariant directly: this is the final hard backstop
against IDM's continuous math ever producing a position update that would
put this vehicle's front bumper past the leader's rear bumper (or past a
red-light stop line) within a single tick, regardless of what acceleration
was computed. It's a discrete-timestep safety clamp layered on top of a
continuous differential-equation model — necessary because Euler
integration over a finite `dt` can otherwise overshoot.

**Usage:** This is the actual position-mutation step every other
distance-based check in the file (`entry_slot_free`, gap-finding in the
simulator) ultimately depends on being accurate.

**Check yourself:**
- Why does clamping `step` also require recomputing `current_speed`, instead of leaving `current_speed` as whatever `idm_accel` implied?
- What would be the observable symptom (in the rendered simulation) if this clamp were removed entirely?

---

```cpp
    if (current_speed < kStuckSpeed && effective_gap < 1e8) {
        stuck_time += dt;
    } else if (current_speed > 1.0) {
        stuck_time = 0.0;
        zone_wait_ = 0.0;
    }

    if (std::abs(lateral_offset - target_lateral_offset) > 1e-3) {
        const double d = target_lateral_offset - lateral_offset;
        const double m = kLateralSpeed * dt;
        lateral_offset += std::abs(d) <= m ? d : (d > 0 ? m : -m);
    }
```

**Meaning:** The stuck-time tracker accumulates `dt` while the vehicle is
nearly stationary (`< kStuckSpeed`) *and* actually has something ahead of it
(`effective_gap < 1e8`, i.e. not the "nothing ahead" sentinel) — resetting
both `stuck_time` and `zone_wait_` once the vehicle is clearly moving again
(`> 1.0` m/s). The second block is the lane-change lateral drift easing
mentioned throughout: while `lateral_offset` hasn't yet reached
`target_lateral_offset`, it steps toward it by up to `kLateralSpeed * dt`
metres per tick, snapping exactly onto the target once within that step
size (avoiding endless tiny overshoot/undershoot oscillation).

**Why:** the `effective_gap < 1e8` condition specifically excludes a
vehicle standing still because there's genuinely nothing to drive toward
(no leader, no light) from counting as "stuck" — `stuck_time` is meant to
measure being *blocked*, not being correctly parked with an open road
ahead. This is the input that eventually feeds `rescue_to()`'s trigger
condition in the simulator.

**Usage:** `stuck_time` is read by the simulator's gridlock-detection logic;
the lateral-offset easing here is the animation half of every `set_lane()`
call made earlier.

**Check yourself:**
- Why does the "reset" branch require `current_speed > 1.0` (a full 1 m/s) rather than resetting the moment speed exceeds `kStuckSpeed` (0.3 m/s) again?

---

```cpp
    if (position_on_road >= current_road->length) {
        const double overshoot = position_on_road - current_road->length;
        const int next_index = route_index_ + 1;
        if (next_index < static_cast<int>(route_.size())) {
            Road* next = route_[next_index];
            int next_lane = lane;
            const std::vector<int> ok = allowed_lanes(next->get_lanes());
            if (std::find(ok.begin(), ok.end(), next_lane) == ok.end()) {
                next_lane = ok.empty() ? 0 : ok.back();
            }
            if (!entry_slot_free(*next, next_lane, overshoot, length_meters())) {
                position_on_road = current_road->length;
                current_speed = 0.0;
                accel_ = 0.0;
                return;
            }
            travel_history_.push_back(current_road->id);
            current_road->remove_vehicle(this);
            route_index_ = next_index;
            current_road = next;
            position_on_road = overshoot;
            lane = next_lane;
            prev_lane_ = -1;
            current_road->add_vehicle(this);
        } else {
            travel_history_.push_back(current_road->id);
            current_road->remove_vehicle(this);
            route_index_ = next_index;
            is_reached_destination_ = true;
            state_ = State::Arrived;
            current_road = nullptr;
            position_on_road = 0.0;
            current_speed = 0.0;
            accel_ = 0.0;
        }
    }
}
```

**Meaning:** Once `position_on_road` reaches or exceeds the current road's
length, a handover is due. `overshoot` is however far past the end the
vehicle traveled. If there's a next road on the route, it picks a valid
lane there the same way as elsewhere, then — this is the same
`entry_slot_free` safety check documented earlier — refuses the handover if
another vehicle already claimed that slot this tick, instead pinning the
vehicle at the very end of its current road with zero speed and returning
early (so it tries again next tick). If the slot is free, the vehicle is
recorded into `travel_history_`, detached from the old road, and reattached
to `next` at the overshoot distance, with `prev_lane_` reset to `-1` (a
fresh road means no "just vacated a lane" state carries over). If there was
no next road, this is the final leg: the vehicle is removed from the road,
marked `Arrived`, and its road/position/speed/acceleration are all zeroed
out — the vehicle is effectively parked in place at the network but no
longer attached to any `Road`.

**Why:** this is the second (and final) place `entry_slot_free` matters,
and it's the direct fix for the exact bug its own comment describes: two
vehicles converging on the same road from different directions in the same
tick. Recall from the top of the file that leaders are picked once per tick
before any vehicle moves — this handover check is what catches a race that
leader-selection alone can't, since it runs live during the move loop
itself, seeing whoever already got there first within this same tick.

**Usage:** This is the mechanism by which a vehicle transitions between
`Road`s along its planned route — the last step of the whole `advance()`
function, and the one that ultimately produces `Arrived` state and removes
a vehicle from active simulation.

**Check yourself:**
- If `entry_slot_free` refuses the handover, the vehicle is pinned at `current_road->length` with `current_speed = 0.0`. On the very next tick, what has to be true for the handover to succeed this time?
- Why is `prev_lane_` reset to `-1` on a successful road handover, but not touched at all on a same-road lane change (`set_lane`)?

---

## `cpp_backend/include/vehicles/VehicleTypes.hpp`

### Lines 1–15: file purpose and includes

```cpp
/**
 * vehicletypes.hpp
 * ----------------
 * Concrete vehicle classes (inheritance + polymorphism) and the
 * VehicleFactory (Factory design pattern) used to spawn them.
 */

#include <memory>
#include <random>
#include <string>
#include <vector>

#include "vehicles/Vehicle.hpp"
```

**Meaning:** A Doxygen-style file banner stating this header's two jobs up
front: the concrete `Vehicle` subclasses, and the Factory. `<random>` is
included for `std::mt19937` (used by `random_type`), `<memory>` for
`std::unique_ptr`. `Vehicle.hpp` is included fully, since every class here
inherits from `Vehicle` and needs its complete definition to override
virtual functions.

**Why:** this is one of the few files in the project with an explicit
"what design patterns live here" comment right at the top — consistent with
the project explicitly naming its patterns elsewhere (Strategy for routing,
Observer for notifications, Singleton for the simulator). Naming the
pattern in the file banner is a documentation choice that pays off when
searching the codebase for "where's the Factory."

**Usage:** Any code that needs to spawn a vehicle by type name (the API
layer, `TrafficSimulator::spawn_locked`) includes this header for
`VehicleFactory::create`.

**Check yourself:**
- Why does this header need to `#include "vehicles/Vehicle.hpp"` fully rather than just forward-declaring `class Vehicle`?

---

### Lines 17–21: `Car`

```cpp
class Car final : public Vehicle {
public:
    using Vehicle::Vehicle;
    std::string type() const override { return "car"; }
};
```

**Meaning:** `Car` inherits from `Vehicle`, marked `final` (no further
subclassing allowed). `using Vehicle::Vehicle;` is a using-declaration that
inherits `Vehicle`'s constructor — `Car` doesn't need to write its own
four-argument constructor; the compiler generates one that just forwards to
`Vehicle`'s. The only override is `type()`, returning `"car"`.

**Why:** `Car` is the baseline vehicle — every hook it doesn't override
(`speed_factor`, `length_meters`, `width_meters`, `congestion_sensitivity`,
`ignores_lights`, `lane_politeness`, `allowed_lanes`) uses `Vehicle`'s
defaults exactly, which is precisely why those defaults in the base class
were chosen to already describe an "ordinary car" (4.5m long, 1.8m wide,
no light-skipping, middling politeness). `final` communicates that nobody
should subclass `Car` further — the type hierarchy here is flat and
intentionally shallow: `Vehicle` → four concrete leaf classes, no deeper.

**Usage:** Constructed via `VehicleFactory::create("car", ...)` or as the
default fallback for any unrecognized type string.

**Check yourself:**
- Why is inheriting the constructor with `using Vehicle::Vehicle;` preferable here to writing out `Car(std::string id, double max_speed, double max_accel, double max_decel) : Vehicle(id, max_speed, max_accel, max_decel) {}` by hand?
- What would `final` prevent someone from doing, and why might that be a reasonable restriction for this class hierarchy?

---

### Lines 23–38: `Bus`

```cpp
class Bus final : public Vehicle {
public:
    using Vehicle::Vehicle;
    std::string type() const override { return "bus"; }
    double speed_factor() const override { return 0.8; }
    double length_meters() const override { return 6.5; }

    std::vector<int> allowed_lanes(int total_lanes) const override {
        if (total_lanes < 3) return Vehicle::allowed_lanes(total_lanes);
        std::vector<int> lanes;
        for (int i = 1; i < total_lanes; ++i) lanes.push_back(i);
        return lanes;
    }
};
```

**Meaning:** `Bus` overrides `speed_factor()` down to `0.8` (20% slower than
baseline — folded straight into `desired_speed_on`'s multiplication) and
`length_meters()` up to `6.5` (versus the 4.5m default, directly affecting
`entry_slot_free`'s clearance math — this is exactly the field responsible
for the car/bus overlap bug mentioned earlier in `entry_slot_free`'s
history). `allowed_lanes` is overridden: on roads with fewer than 3 lanes,
it just falls back to the base class's "every lane is fine" behavior
(`Vehicle::allowed_lanes`, an explicit call to the base implementation from
inside the override — not a re-implementation of the same loop); on 3+
lane roads, it excludes lane `0` (the innermost, per the header's "0 is the
innermost" comment), returning only lanes `1` through `total_lanes - 1`.

**Why:** the comment "On 3+ lane roads buses keep out of the innermost
(fast) lane" encodes a real-world traffic rule directly as a lane-index
restriction — and it's a clean illustration of why `allowed_lanes` exists
as a hook at all: it's the mechanism by which per-vehicle-type lane rules
get enforced everywhere lanes are chosen (`request_route`'s implicit lane 0
default aside, `transfer_to_road`, `rescue_to`, and the road-handover step
of `advance()` all call `allowed_lanes()` before settling a vehicle onto a
lane). Reusing `Vehicle::allowed_lanes(total_lanes)` for the `< 3` case
instead of duplicating the "all lanes" loop is good hygiene — one
canonical definition of "no restriction," inherited rather than copied.

**Usage:** Every lane-assignment decision in `Vehicle.cpp` funnels through
this override for any vehicle whose `type()` is `"bus"`.

**Check yourself:**
- Why does the `< 3` branch explicitly call `Vehicle::allowed_lanes(total_lanes)` instead of just writing the identical loop again inline?
- If a `Bus` somehow ended up assigned to lane `0` on a 4-lane road (say, from before this rule existed) and then `advance()` ran a road-handover with a 4-lane next road, what would `allowed_lanes` cause to happen?

---

### Lines 40–52: `EmergencyVehicle`

```cpp
class EmergencyVehicle final : public Vehicle {
public:
    using Vehicle::Vehicle;
    std::string type() const override { return "emergency"; }
    double speed_factor() const override { return 1.35; }
    double congestion_sensitivity() const override { return 0.25; }
    bool ignores_lights() const override { return true; }
    double length_meters() const override { return 5.0; }
    double lane_politeness() const override { return 0.0; }
};
```

**Meaning:** `EmergencyVehicle` overrides five hooks: `speed_factor` up to
`1.35` (35% faster than baseline), `congestion_sensitivity` down to `0.25`
(only a quarter as affected by road congestion — see `desired_speed_on`'s
formula, which passes this straight into `Road::effective_speed_ms`),
`ignores_lights` flipped to `true`, `length_meters` at `5.0` (between a car
and a bus), and `lane_politeness` down to `0.0`.

**Why:** every one of these maps directly onto real emergency-vehicle
behavior, and every one connects to a specific mechanism already covered:
`ignores_lights() == true` is checked directly in `advance()`'s light-branch
guard (`!ignores_lights() && ...`), so this vehicle skips the entire
red/amber stop-line logic outright. `lane_politeness() == 0.0` is the MOBIL
hook from the header — the inline comment "sirens don't negotiate" says
plainly that in the (simulator-side) incentive computation, this vehicle's
lane changes weight *only* its own benefit, giving zero consideration to
how much worse a change makes things for the vehicle it cuts in front of.
Low `congestion_sensitivity` means this vehicle's desired speed barely
drops even on a jammed road, modeling "weaves through congestion" from the
class comment.

**Usage:** Spawned via `VehicleFactory::create("emergency", ...)`, and
because `random_type` (below) only picks `"emergency"` 4% of the time,
these are rare in an organically-populated simulation.

**Check yourself:**
- Trace `lane_politeness() == 0.0` from this declaration to the MOBIL incentive comparison discussed in the `Vehicle.hpp` section — since the actual comparison isn't implemented in this file, what can you say for certain about its effect just from knowing this value is the *minimum* of the politeness range, versus having to guess at the simulator's formula?
- Why does `EmergencyVehicle` not override `allowed_lanes()` — is there a real-world reason emergency vehicles wouldn't need a lane restriction the way buses and motorbikes do?

---

### Lines 54–69: `Motorbike`

```cpp
class Motorbike final : public Vehicle {
public:
    using Vehicle::Vehicle;
    std::string type() const override { return "motorbike"; }
    double speed_factor() const override { return 0.95; }
    double congestion_sensitivity() const override { return 0.6; }
    double length_meters() const override { return 2.0; }
    double width_meters() const override { return 0.8; }

    std::vector<int> allowed_lanes(int total_lanes) const override {
        if (total_lanes < 3) return Vehicle::allowed_lanes(total_lanes);
        return {0, 1};
    }
};
```

**Meaning:** `Motorbike` is slightly slower than baseline (`speed_factor`
`0.95`), moderately less congestion-sensitive (`0.6`, "nimble in
congestion" per the class comment), much smaller (`2.0m` long, `0.8m` wide
versus the `4.5m`/`1.8m` defaults). Its `allowed_lanes` override is the
mirror image of `Bus`'s: on 3+ lane roads it's confined to `{0, 1}` — the
two *innermost* lanes — using a brace-init list to return exactly those two
values rather than building the list with a loop.

**Why:** `width_meters()` is the one hook only `Motorbike` overrides among
all four concrete classes — every other type keeps the `1.8m` default. It's
declared on `Vehicle` (line 58 of `Vehicle.hpp`) but note from the earlier
survey of `Vehicle.cpp` that width doesn't appear anywhere in
`entry_slot_free`, `idm_accel`, or `advance()` — the collision/gap
math is entirely along-road (length-based), never across-lane (width-based).
That's one more angle on the lane-locking limitation: even a vehicle type
explicitly narrower than a full lane has no mechanism here to actually use
that narrowness (e.g. sharing a lane side-by-side with another vehicle) —
`width_meters()` currently only matters for rendering, the same way
`lateral_offset` does. `allowed_lanes` returning a fixed `{0, 1}` (rather
than a loop like `Bus`'s) is fine specifically because it's a hardcoded
two-lane rule, not a "everything except the first N" rule — if it needed to
scale with `total_lanes`, a loop would be necessary instead.

**Usage:** Same call sites as `Bus`'s `allowed_lanes` override — every lane
assignment in `Vehicle.cpp` for a `"motorbike"`-typed vehicle.

**Check yourself:**
- `width_meters()` is overridden here but never read by any collision or gap-finding logic in `Vehicle.cpp`. What would need to change in `entry_slot_free` (or a sibling function) for motorbikes to actually be able to share a lane side-by-side, and how does that connect to the "no continuous lateral position" limitation?
- Why does `Motorbike::allowed_lanes` return a fixed `{0, 1}` instead of `Bus`'s pattern of "every lane except the excluded ones"?

---

### Lines 71–92: `VehicleFactory`

```cpp
class VehicleFactory {
public:
    static std::unique_ptr<Vehicle> create(const std::string& type,
                                           const std::string& id,
                                           double max_speed = 16.7,
                                           double max_accel = 3.0,
                                           double max_decel = 4.5);
    static std::string random_type(std::mt19937& rng);
    static std::string canonical_type(const std::string& type);
};
```

**Meaning:** `VehicleFactory` has no data members and no instances are ever
constructed — every member is `static`, making the class purely a
namespacing device around three related functions (a common alternative to
a plain namespace when a class name reads better at call sites, e.g.
`VehicleFactory::create(...)`). `create` returns a `unique_ptr<Vehicle>`
(base-class pointer, polymorphic ownership) built from a type string and an
id, with default speed/accel/decel values baked in as default arguments.
`random_type` takes a `std::mt19937&` (a Mersenne Twister PRNG, passed by
reference so the *caller* owns and seeds it — the factory doesn't manage
randomness state itself) and returns a weighted-random type name.
`canonical_type` maps an arbitrary string to one of the four real type
names.

**Why:** this is the Factory pattern named in the file banner — centralizing
"given a type string, build the right concrete class" in one place means
every caller (API request handling, organic spawning, tests) gets identical
behavior for unrecognized type strings, rather than each call site
independently deciding what "unknown type" should fall back to. Taking
`std::mt19937&` rather than owning its own RNG is what keeps `random_type`
reproducible under a fixed seed — if the factory owned a static RNG
internally, two runs of the whole simulator wouldn't produce the same
random vehicle mix unless that internal state were also carefully reset,
whereas an externally-owned, externally-seeded generator is trivially
reproducible by construction.

**Usage:** `create` is called from `TrafficSimulator::spawn_locked` and any
API endpoint that spawns a vehicle by name; `random_type` is used for
organic spawning where no specific type was requested; `canonical_type` — as
its own doc comment on line 89–91 explains — exists specifically because
`spawn_locked` needs to build a vehicle's `id` string (which embeds a type
prefix, per the earlier "`<type><counter>`" discussion in `Vehicle.cpp`)
*before* the `Vehicle` object exists to ask `type()` of, so it needs its own
routine that's guaranteed to produce the exact same fallback `create()`
would have used.

**Check yourself:**
- Why does `VehicleFactory` use `static` member functions in a class rather than free functions in a namespace — what's actually different between the two approaches here?
- Trace why `canonical_type` needs to exist at all as a *separate* function from `create`, given that they must agree — what would go wrong if `spawn_locked` just guessed the id prefix independently instead of calling `canonical_type`?

---

## `cpp_backend/src/vehicles/VehicleTypes.cpp`

### Lines 3–8: `canonical_type`

```cpp
std::string VehicleFactory::canonical_type(const std::string& type) {
    if (type == "bus") return "bus";
    if (type == "emergency") return "emergency";
    if (type == "motorbike") return "motorbike";
    return "car";  // default, and explicit "car"
}
```

**Meaning:** A simple whitelist check: exact string matches for the three
non-default types, `"car"` for everything else — including both an
unrecognized string *and* the explicit string `"car"` itself, per the
trailing comment.

**Why:** this mirrors `create()`'s own if-chain (below) exactly, term for
term, which is precisely the guarantee the header promised: whatever string
gets passed, `canonical_type` and `create` will agree on which of the four
concrete classes it maps to. If they ever diverged (e.g. someone adds a
fifth type to `create` but forgets to add the matching branch here), a
vehicle's id prefix would no longer match its actual `type()` string — which
the header's `Vehicle.cpp` comment already flagged as something the
frontend's id-reconstruction logic depends on staying correct.

**Usage:** Called by `TrafficSimulator::spawn_locked` before constructing
the `Vehicle`, to build the id string's type prefix.

**Check yourself:**
- If a new vehicle type `"taxi"` were added to `create()` but this function's if-chain were left unchanged, what specific, hard-to-notice bug would result the next time a taxi was spawned?

---

### Lines 10–28: `create`

```cpp
std::unique_ptr<Vehicle> VehicleFactory::create(
    const std::string& type, const std::string& id,
    double max_speed, double max_accel, double max_decel)
{
    if (type == "bus") return std::make_unique<Bus>(id, max_speed, max_accel, max_decel);
    if (type == "emergency") return std::make_unique<EmergencyVehicle>(id, max_speed, max_accel, max_decel);
    if (type == "motorbike") return std::make_unique<Motorbike>(id, max_speed, max_accel, max_decel);
    return std::make_unique<Car>(id, max_speed, max_accel, max_decel);
}
```

**Meaning:** Note the default arguments (`max_speed = 16.7`, etc.) declared
in the header are *not* repeated here — C++ requires default arguments to
appear only in the declaration seen by callers (typically the header), never
duplicated in the definition, otherwise it's a compile error. The body is a
plain if-chain matching type strings to `std::make_unique<ConcreteClass>(...)`
calls, implicitly converting each concrete `unique_ptr<Bus>` /
`unique_ptr<EmergencyVehicle>` / etc. to the returned `unique_ptr<Vehicle>`
(legal because each concrete class publicly inherits `Vehicle`, and because
`~Vehicle()` is virtual, per the earlier discussion — the exact reason that
destructor needed to be virtual in the first place).

**Why:** `std::make_unique` (rather than `new Bus(...)` wrapped in a
`unique_ptr` constructor) is the standard modern-C++ idiom — it's
exception-safe (no window where a raw `new` succeeds but the `unique_ptr`
wrapping throws before taking ownership) and avoids writing the type name
twice.

**Usage:** This is the single choke point every vehicle in the entire
simulator is constructed through — `learn/07-simulation-core.md`'s spawn
logic, and every test that constructs a vehicle by type name, all end up
here.

**Check yourself:**
- Why is it a compile error to repeat the default-argument values (`= 16.7`, etc.) in this `.cpp` definition when they're already given in the header declaration?
- What C++ language feature specifically makes `return std::make_unique<Bus>(...)` (which produces a `unique_ptr<Bus>`) legal as the return value of a function declared to return `unique_ptr<Vehicle>`?

---

### Lines 30–37: `random_type`

```cpp
std::string VehicleFactory::random_type(std::mt19937& rng) {
    std::uniform_int_distribution<int> dist(0, 99);
    const int roll = dist(rng);
    if (roll < 68) return "car";
    if (roll < 84) return "motorbike";
    if (roll < 96) return "bus";
    return "emergency";
}
```

**Meaning:** Draws a uniform integer in `[0, 99]` from the caller-supplied
generator, then buckets it: `0–67` (68%) → car, `68–83` (16%) → motorbike,
`84–95` (12%) → bus, `96–99` (4%) → emergency. The cumulative thresholds
(`68`, `84`, `96`) are what encode each bucket's width, since each `if`
only needs to check the upper edge given the ones above it already failed.

**Why:** this is a manual weighted-random selection built from a single
`uniform_int_distribution` rather than, say, `std::discrete_distribution`
(which would take the weights `{68, 16, 12, 4}` directly and be arguably
more self-documenting). The chosen approach works fine and keeps the
function free of an extra distribution object, but it does mean the actual
weights (68/16/12/4) have to be reverse-engineered from the threshold
differences rather than read directly off the code — worth knowing if
you're the one tuning the vehicle mix later.

**Usage:** Called wherever the simulator spawns a vehicle organically
without the API or test caller specifying a type — see
`learn/07-simulation-core.md` for the spawn loop that calls this per new
vehicle.

**Check yourself:**
- What are the actual percentage weights for each of the four types, computed from the threshold values `68`, `84`, `96`, and `100`?
- Rewrite this function's logic (on paper, not in the file) using `std::discrete_distribution<int>({68, 16, 12, 4})` instead — what would the resulting code gain or lose in clarity compared to the threshold-chain version actually in the file?

---

## Tying it back to the known limitations

Two of the project's four stated limitations have a concrete, named
location in this module:

- **"Vehicles are locked to discrete lanes (no continuous lateral
  position)."** The mechanism is `Vehicle::lane` (an `int`, `Vehicle.hpp`
  line 39) plus every function that branches on it with an exact-equality
  check — `entry_slot_free`'s `o->lane != lane` guard being the clearest
  example. `lateral_offset`/`target_lateral_offset` look like they might be
  the escape hatch, but they're driven only by `set_lane()`'s cosmetic
  "animate across" logic and never participate in any gap, collision, or
  MOBIL-safety computation — a lane change is an instantaneous integer flip
  physically, and only a smooth slide visually.
- **"Gridlocks are handled reactively... rather than prevented
  proactively."** `zone_wait_`/`is_forced()`/`grant_forced_clearance()` and
  `rescue_to()` are both reactive-by-construction: impatience only starts
  accumulating once a vehicle is already waiting, and `rescue_to()` only
  ever fires once a vehicle is already judged stuck — there's no mechanism
  in this file that looks ahead and avoids creating the jam in the first
  place. That decision logic itself lives in the simulator
  (`learn/07-simulation-core.md`); what's here is the vehicle-side state
  those decisions read and write.

---

## Check yourself — module synthesis

- A `Car` and a `Bus` both approach the same single-lane road entry at the
  same tick. Trace, end to end, every function from this module that runs
  before both vehicles' positions are finalized, and identify the one
  check that guarantees they don't end up overlapping.
- Suppose a fifth vehicle type, `Taxi`, needs to be added: normal speed,
  normal size, but *ignores* congestion sensitivity entirely and always
  uses the innermost lane on 3+ lane roads. List every file and every
  specific line/function in this module that would need to change, in the
  order you'd change them.
- `Vehicle::advance()` computes `state_` before running the physics for the
  tick, and the header comment on the `State` enum insists this distinction
  matters for `/api/state`. If you deleted `State` entirely and had the
  frontend infer "why is this car stopped" purely from `current_speed == 0`,
  what specific scenario would become impossible to distinguish?
</content>
