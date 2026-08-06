# Module 03 — Traffic Control Primitives

**Files covered:**
[`cpp_backend/include/core/TrafficLight.hpp`](../cpp_backend/include/core/TrafficLight.hpp) (65 lines),
[`cpp_backend/src/core/TrafficLight.cpp`](../cpp_backend/src/core/TrafficLight.cpp) (77 lines)

`TrafficLight` is small — one header, one `.cpp`, no inheritance, no virtual
functions. But it embeds a design decision that shapes everything downstream
of it: a signal's colour is computed, not stored. There is no `tick()` on
this class, no timer counting down, no mutable "current state" field that the
[simulation tick loop](07-simulation-core.md) has to remember to advance.
Every question you can ask a `TrafficLight` — "what colour right now?", "how
long until it changes?" — is answered by plugging a timestamp into a formula.
Keep that in mind through the whole module; almost every choice below traces
back to it.

---

### `cpp_backend/include/core/TrafficLight.hpp`, lines 1–4: include guard and dependency

```cpp
#ifndef TRAFFICLIGHT_H
#define TRAFFICLIGHT_H

#include <string>
```

**Meaning:** The classic include-guard pair — `#ifndef`/`#define` — prevents
the compiler from processing this header's contents twice if it's
transitively `#include`d more than once in the same translation unit (a very
real risk here, since `Intersection.hpp` includes it and lots of files
include `Intersection.hpp`). `<string>` is pulled in because one member
function below takes a `const std::string&`.

**Why:** Nothing unusual — this is boilerplate every header in the project
needs. Worth noting only because the guard macro is `TRAFFICLIGHT_H`, not
something namespaced like `CORE_TRAFFICLIGHT_HPP` — a minor but real
collision risk if the codebase ever grows a second `TrafficLight`-named type
elsewhere, though at this project's size it's a non-issue in practice.

**Usage:** Every file that needs to know what colour a light is showing —
`Intersection.hpp`, `TrafficEvent.cpp`, `Vehicle.cpp` — includes this header
directly or transitively.

---

### Lines 6–24: the block comment — read this before the class

```cpp
// Signal cycle for one junction.
//
// The colour is a PURE FUNCTION of simulation time, not a countdown that has to
// be ticked. That matters for three reasons:
//
//   * Opposing approaches must be able to disagree. A junction runs `groups`
//     phase groups that take turns, so while group 0 is green every other group
//     is red. A single stored `state` can only say one thing at a time, so with
//     it a junction is either open to everyone or closed to everyone at once.
//   * Every junction must not switch at the same instant. `phase_offset` is
//     derived from the junction id, so the city's lights spread across the
//     cycle instead of blinking in unison.
//   * A stateless cycle cannot drift. Asking at any sim time gives the same
//     colour, so /api/state and the vehicle physics can never disagree about
//     what a light was showing.
```

**Meaning:** This is documentation, not code, but it's load-bearing
documentation — it states the class invariant up front: `state_at(group,
sim_time)` must be a pure function, meaning same inputs always produce the
same output, with no hidden internal state consulted or mutated along the
way. Three separate justifications are given, each mapping to a specific
member you'll meet below: `groups` (multiple phase groups instead of one
flag), `phase_offset` (desynchronized junctions), and the "pure function"
property itself (no drift between two callers).

**Why:** Contrast this with the obvious naive design — a light object that
owns a countdown timer, gets `update(dt)` called on it every tick, and
mutates an internal `current_state` field. That design is intuitive but has
a real failure mode: two different subsystems asking "what colour is this
light?" at logically the same simulation moment could get different answers
if one asks before that tick's `update()` runs and one asks after, or if a
timer accumulates floating-point error over thousands of ticks. The comment's
third bullet — "`/api/state` and the vehicle physics can never disagree" —
names the two real call sites that would otherwise be vulnerable to exactly
that class of bug: the WebSocket state broadcast (see
[`learn/09-server-and-entrypoint.md`](09-server-and-entrypoint.md)) and
`Vehicle`'s stop-line logic (`Vehicle.cpp`, using `signal_state_for` — see
[`learn/05-vehicles.md`](05-vehicles.md)).

**Usage:** Everything in this module is in service of this one guarantee.
When you read `state_at` below, check that it never reads or writes anything
other than its parameters and the object's immutable configuration
(`green_dur`, `amber_dur`, `all_red_dur`, `groups`, `phase_offset`) — it
does.

---

### Lines 25–31: the class and its data members

```cpp
class TrafficLight {
  private:
    double green_dur;     // seconds of green per group
    double amber_dur;     // seconds of amber per group
    double all_red_dur;   // clearance interval at the end of each group's slot
    int groups;           // number of phase groups taking turns
    double phase_offset;  // seconds, derived from the junction id
```

**Meaning:** Five `private` data members, all set at construction (you'll
see below that only `groups` and `phase_offset` have public setters after
that — `green_dur`, `amber_dur`, `all_red_dur` are otherwise fixed once
built, aside from the one narrow exception at the bottom of the file).
`green_dur`/`amber_dur`/`all_red_dur` are per-group durations in seconds.
`groups` is how many phase groups this junction's approaches are split into.
`phase_offset` is a per-instance time shift.

**Why:** Using `double` for durations and offsets — rather than, say,
integer milliseconds — matches the rest of the codebase's convention of
representing simulation time as a floating-point second count (you'll see
`sim_time` as a `double` everywhere this class is called). It also means a
duration like "2.5 seconds of amber" doesn't require inventing a sub-second
unit. Keeping all five members `private` with no direct field access from
outside enforces the "pure function of *these specific* inputs" invariant —
nothing outside the class can quietly poke a new value into `green_dur`
mid-cycle without going through `set_green_duration`, which is the one
sanctioned mutation path (see near the bottom of this module).

**Usage:** `Intersection` owns one `TrafficLight` per signalized junction via
`std::unique_ptr<TrafficLight> light_system` (see
[`learn/02-core-graph-model.md`](02-core-graph-model.md) for `Intersection`
itself). These five members are exactly what `MapLoader` reads out of the
map's JSON when a junction is signalized, and exactly what gets written back
out when the map is saved.

---

### Lines 33–36: the `State` enum

```cpp
  public:
    // Colours, matching the numbers /api/state has always sent the frontend.
    enum State : unsigned int { Red = 0, Amber = 1, Green = 2 };
```

**Meaning:** An unscoped enum (not `enum class`) named `State`, with its
underlying type explicitly fixed to `unsigned int` and explicit numeric
values assigned to each enumerator: `Red = 0`, `Amber = 1`, `Green = 2`.
Because it's unscoped, `TrafficLight::Red` is usable directly as an
`unsigned int` without an explicit cast — you'll see `signal_state_for`
returning `unsigned int` rather than `TrafficLight::State` for exactly this
reason.

**Why:** The comment is explicit about *why* the numbers are pinned down
rather than left to the compiler's default (0, 1, 2 in declaration order,
which happens to match here — but pinning them explicitly means a future
edit that reorders the enumerators, say for readability, can't silently
change the wire values). "matching the numbers `/api/state` has always sent
the frontend" says this enum is effectively a wire protocol: the frontend
(plain JS, no shared type definitions with C++) expects `0/1/2` for
red/amber/green, so these values are contractual, not incidental. Choosing
`enum class State` instead — the more idiomatic modern C++ choice — would
have forced explicit casts (`static_cast<int>(State::Red)`) everywhere this
value crosses into JSON serialization or gets compared against a raw
integer, which the codebase evidently decided wasn't worth the friction here.

**Usage:** This is the type `state_at` returns and what `Vehicle.cpp`
compares against (`light == TrafficLight::Red`, `light ==
TrafficLight::Amber`) to decide whether to stop or go — see
[`learn/05-vehicles.md`](05-vehicles.md). It's also almost certainly copied
verbatim into the frontend's own JS constants; if you ever need to add a
fourth signal colour, this enum, the frontend's mirror of it, and every
`switch`/comparison against these three values are the coordinated change.

---

### Lines 37–39: the two constructors

```cpp
    TrafficLight();
    // Light loaded from the map file: green/amber/all-red durations and groups.
    TrafficLight(double green, double amber, double all_red, int groups = 2);
```

**Meaning:** Two declared constructors. The default constructor takes no
arguments. The second takes four parameters, the last (`groups`) with a
default argument of `2`, so it can be called with three arguments
(`TrafficLight(25.0, 3.0, 2.0)`) and still compile.

**Why:** Two constructors rather than one with all-defaulted parameters
(`TrafficLight(double green = 25.0, double amber = 3.0, ...)`) is a
readability choice: the no-argument form reads as "give me the standard
default light" at a call site, while the four-argument form reads as "build
a specifically-configured light" — you'll see in the `.cpp` that the
zero-argument constructor is implemented by delegating to the four-argument
one with named constants, so there's exactly one place the actual default
numbers live.

**Usage:** The default constructor is what a test file reaches for when it
needs *a* light without caring about its exact timing (see `TrafficLight a,
b;` in the test excerpts below). The four-argument constructor is what
`MapLoader` calls when a map file specifies explicit `green`/`amber`/`red`
values for a junction (see
[`learn/08-supporting-systems.md`](08-supporting-systems.md)).

---

### Lines 41–46: config getters and the `groups` setter

```cpp
    // Read the phase durations (config), e.g. for saving the map or the UI.
    double get_green_duration() const { return green_dur; }
    double get_yellow_duration() const { return amber_dur; }
    double get_red_duration() const { return all_red_dur; }
    int get_groups() const { return groups; }
    void set_groups(int n);
```

**Meaning:** Four inline (defined in the class body, implicitly `const`
member functions that just return a stored field) getters and one
non-`const` setter declared but not defined here (its body is in the
`.cpp`). Note the naming mismatch: the field is `amber_dur`, but the getter
is `get_yellow_duration()` — "amber" internally, "yellow" at the public API
boundary.

**Why:** Being `const`, the getters can be called through a `const
TrafficLight&` or `const TrafficLight*`, which matters because several call
sites (like the test suite's `m->light_system->get_green_duration()`) reach
these through a `const Intersection*`. The amber/yellow naming split is a
minor rough edge rather than a deliberate design choice — most likely
"amber" was the term used when the class was written (it's the more
common term outside North America) and "yellow" crept in at the API layer
to match whatever the frontend or map-file schema calls it. It's not wrong,
just inconsistent, and worth noticing so it doesn't trip you up grepping the
codebase for one term and missing the other.

**Usage:** These are read, not by the simulation loop for physics, but by
anything that needs to *report* a light's configuration rather than its
current colour — the map-saving path and the frontend UI (per the comment).
`set_groups` — the one setter in this block — reconfigures how many phase
groups a light cycles through; you'll see its implementation clamps the
input, so it can't be used to put the object into an invalid state.

---

### Lines 48–51: the phase-offset pair

```cpp
    // Spread this junction across the cycle. Derived from the id, so reloading
    // the same map reproduces exactly the same timings.
    void set_phase_offset_from(const std::string& id);
    double get_phase_offset() const { return phase_offset; }
```

**Meaning:** `set_phase_offset_from` takes the junction's string id and
computes `phase_offset` from it (implementation below). `get_phase_offset`
is a plain `const` getter.

**Why:** This is the second of the three points from the class-level
comment: without a per-junction offset, every `TrafficLight` in the city
that shares the same `green_dur`/`amber_dur`/`all_red_dur`/`groups`
configuration would be perfectly in phase with every other one — all
turning green and red at the exact same simulated instant, which looks
obviously wrong in the frontend and isn't how real signal timing works.
Deriving the offset from the *id* rather than, say, an incrementing counter
assigned at load order, or a random draw at construction time, is what makes
the offset reproducible: reload the same map file, get the same id strings,
get the same offsets, every time — no hidden dependency on load order or a
random seed that would make a specific bug (the comment's example: "a bug at
t = 137 s") non-reproducible between runs.

**Usage:** `MapLoader` calls `set_phase_offset_from` once per junction
during map load (see
[`learn/08-supporting-systems.md`](08-supporting-systems.md)), passing the
junction's id. After that, `phase_offset` is folded into every `state_at`
and `remaining_at` call transparently — callers never see or pass the offset
themselves.

---

### Lines 53–56: `state_at` — the core query

```cpp
    // Colour shown to approach `group` at `sim_time`. A group outside
    // [0, groups) is red, so an approach that never got a group assigned fails
    // closed instead of being waved through.
    State state_at(int group, double sim_time) const;
```

**Meaning:** Declares the signature this whole module has been building
toward: given a phase group number and a simulation timestamp, return the
`State`. `const` — as promised, this cannot mutate the object.

**Why:** The comment states a safety property directly: "fails closed." A
`Road` that hasn't been assigned to any phase group carries `signal_group ==
-1` (per the test excerpts you'll see later), and `-1` is outside `[0,
groups)` for any valid `groups` value — so passing it here returns `Red`,
not an out-of-bounds read or an accidental match. This matters because the
alternative failure mode — treating an unassigned or malformed group number
as "group 0" — would silently wave unassigned traffic through a live
junction, which is a correctness bug that could contribute to the
"junction overlap" problem flagged as a known project limitation. Failing
closed instead of guessing is the safer default whenever the input is
ambiguous.

**Usage:** `Intersection::signal_state_for` (in `Intersection.cpp`) is the
sole call site, forwarding straight through: `return
light_system->state_at(group, sim_time);`. `Vehicle.cpp` calls
`next_node->signal_state_for(current_road->signal_group, sim_time)` to
decide whether to stop at a junction — see
[`learn/05-vehicles.md`](05-vehicles.md) — and the WebSocket state broadcast
almost certainly calls the same path per junction per group to report
colours to the frontend (see
[`learn/09-server-and-entrypoint.md`](09-server-and-entrypoint.md)).

---

### Lines 58–59: `remaining_at` — the UI countdown

```cpp
    // Seconds until this group's colour changes next (UI countdown).
    double remaining_at(int group, double sim_time) const;
```

**Meaning:** A second query, also `const`, also a pure function of `(group,
sim_time)`, returning how many seconds remain until that group's displayed
colour next changes.

**Why:** This is a separate function rather than something the caller
derives from `state_at` because the "seconds until next change" computation
needs to know *where in the cycle* the group currently is, including the
case where the group's slot hasn't started yet — information `state_at`
deliberately doesn't expose (it only returns the colour, not the phase
position). Keeping both functions pure and independent (even though they
share nearly identical setup logic — see the `.cpp`) preserves the same
no-drift guarantee for the countdown display that `state_at` gives the
colour itself.

**Usage:** `Intersection::remaining_at` — the exact same one-line-forward
pattern as `signal_state_for` — is the only call site in `Intersection.cpp`.
The doc comment says exactly who consumes it: a UI countdown, i.e. the
frontend showing "3s" next to a red or green light, driven by
[`learn/09-server-and-entrypoint.md`](09-server-and-entrypoint.md)'s state
broadcast.

---

### Lines 61–62: `set_green_duration` — the one mutator

```cpp
    // Behaviour for TrafficEvent: a broken light runs a shorter green.
    void set_green_duration(double time);
```

**Meaning:** A setter for `green_dur`, the only way to mutate a
`TrafficLight` after construction besides `set_groups`.

**Why:** The comment names the caller directly: `TrafficEvent`. This is the
one place `TrafficLight`'s "config is fixed after construction" story bends
— a light-failure event needs to *change* a live light's timing, not just
report it. But note it still doesn't break the pure-function guarantee: it
changes what function `state_at` computes going forward, it doesn't
introduce any state that depends on *when* `set_green_duration` was called
relative to the current cycle position. You'll see in the `.cpp` that this
is exactly why no special "cut the current phase short" logic is needed —
the next `state_at` call just evaluates the new formula.

**Usage:** `TrafficLightEvent::apply` (in
[`cpp_backend/src/events/TrafficEvent.cpp`](../cpp_backend/src/events/TrafficEvent.cpp))
calls this to scale a junction's green duration down by
`kBrokenGreenScale = 0.2` when a "broken light" event starts, and
`TrafficLightEvent::expire` calls it again to scale back up by the inverse
when the event ends. See
[`learn/06-events-observer.md`](06-events-observer.md) for how `TrafficEvent`
subclasses are dispatched and expired.

---

That's the whole header — five members, two constructors, four
configuration getters, two setters, and two query functions. Now the
implementation.

---

### `cpp_backend/src/core/TrafficLight.cpp`, lines 1–10: includes and defaults

```cpp
#include "core/TrafficLight.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

// Default cycle (seconds), matching the map file's default values.
static const double DEFAULT_GREEN   = 25.0;
static const double DEFAULT_AMBER   = 3.0;
static const double DEFAULT_ALL_RED = 2.0;
```

**Meaning:** Includes the class's own header first (standard practice — it
verifies the header is self-sufficient), then three standard-library
headers: `<algorithm>` for `std::max`/`std::clamp`, `<cmath>` for
`std::fmod`, `<functional>` for `std::hash`. Three `static const double`
file-scope constants hold the default cycle timing: 25 s green, 3 s amber, 2
s all-red.

**Why:** `static` here gives these constants internal linkage — each
translation unit that might define its own `DEFAULT_GREEN` (unlikely, but
possible in a large codebase) can't collide with this one at link time. The
comment "matching the map file's default values" tells you these numbers
aren't arbitrary — they're kept in sync with whatever `MapLoader` treats as
"unspecified" in a map's JSON, so a junction that doesn't override its
timing gets the same values whether the map file omits the fields or this
class's default constructor is used directly.

**Usage:** Consumed exactly once, by the default constructor immediately
below.

---

### Lines 12–20: the two constructor bodies

```cpp
TrafficLight::TrafficLight()
    : TrafficLight(DEFAULT_GREEN, DEFAULT_AMBER, DEFAULT_ALL_RED, 2) {}

TrafficLight::TrafficLight(double green, double amber, double all_red, int groups)
    : green_dur(std::max(1.0, green)),
      amber_dur(std::max(0.0, amber)),
      all_red_dur(std::max(0.0, all_red)),
      groups(std::clamp(groups, 1, 4)),
      phase_offset(0.0) {}
```

**Meaning:** The no-argument constructor uses a **delegating constructor**
(`: TrafficLight(...)`) — the C++11 feature that lets one constructor call
another constructor of the same class in its member-initializer list — to
forward straight to the four-argument constructor with the three
`DEFAULT_*` constants and `groups = 2`. The four-argument constructor
initializes every member in its member-initializer list: `green_dur` is
clamped to a floor of `1.0` via `std::max`, `amber_dur` and `all_red_dur`
are floored at `0.0`, `groups` is clamped into `[1, 4]` via `std::clamp`,
and `phase_offset` starts at `0.0` (until/unless `set_phase_offset_from` is
called later).

**Why:** The delegating constructor means `DEFAULT_GREEN`/`DEFAULT_AMBER`/
`DEFAULT_ALL_RED` are used in exactly one place — if the defaults ever
change, there's one line to edit, not two constructor bodies to keep in
sync. The clamps are defensive: `green_dur`'s floor of `1.0` (not `0.0`)
guarantees a signal always spends *some* nonzero time green — a light with
`green_dur = 0` would be functionally broken (nobody would ever be waved
through), so `1.0` second is the pragmatic floor. `amber_dur` and
`all_red_dur` can legitimately be `0.0` (a junction with no amber phase or
no clearance interval isn't invalid, just aggressive), so they're only
floored to reject negative input, not raised to a positive minimum. `groups`
being clamped to `[1, 4]` bounds the phase-group count into what the rest of
the codebase — and the map format — actually supports; this is also the
reason `state_at`'s "group outside `[0, groups)` is red" guard from the
header comment is safe: `groups` itself can never be zero or negative, so
`[0, groups)` is never an empty or invalid range.

**Usage:** Every `TrafficLight` in the system — whether default-constructed
by a test, or built with explicit values by `MapLoader` — passes through
these clamps, so no code downstream (`state_at`, `remaining_at`) ever has to
defend against a zero or negative duration or an out-of-range group count.

---

### Line 22: `set_groups`

```cpp
void TrafficLight::set_groups(int n) { groups = std::clamp(n, 1, 4); }
```

**Meaning:** A one-line function body: clamp the argument into `[1, 4]`,
same as the constructor, and assign.

**Why:** Reusing the exact same clamp range as the constructor keeps the
invariant "`groups` is always in `[1, 4]`" true for the object's entire
lifetime, not just at construction — without this, a caller could set
`groups` to `0` or a huge number post-construction and violate the
assumption `state_at` relies on.

**Usage:** Called if a junction's phase-group count needs to change after
load — for instance, from a map-editing tool (see
[`learn/08-supporting-systems.md`](08-supporting-systems.md) for
`MapEditor`) reconfiguring an existing junction's signal layout.

---

### Lines 24–30: `set_phase_offset_from` — deriving desync from the id

```cpp
// Hash the id into an offset spread over one full minute. Using the id rather
// than a counter or a random draw means a saved map reloads with the identical
// timings, which is what makes a bug at t = 137 s reproducible.
void TrafficLight::set_phase_offset_from(const std::string& id) {
    const std::size_t h = std::hash<std::string>{}(id);
    phase_offset = static_cast<double>(h % 1000) / 1000.0 * 60.0;
}
```

**Meaning:** `std::hash<std::string>{}(id)` constructs a temporary
`std::hash<std::string>` function object and immediately invokes it on
`id`, producing a `std::size_t` (an implementation-defined, effectively
unpredictable-looking integer derived from the string's contents — same
input string always hashes to the same output, but there's no simple
relationship between similar strings and similar hashes). `h % 1000` reduces
that to a value in `[0, 999]`, cast to `double` and scaled: divide by
`1000.0` to get a fraction in `[0, 1)`, multiply by `60.0` to spread that
fraction over a 60-second range. The result is stored in `phase_offset`.

**Why:** The comment states the reasoning plainly — deriving the offset from
the junction's *id string* (deterministic hash) rather than a load-order
counter or `rand()` is what makes the offset reproducible across reloads of
the same map. `% 1000` then scaling to 60 seconds means the offset is spread
reasonably evenly across a full minute regardless of what the actual
green/amber/red cycle length is — it doesn't try to align to the light's own
cycle length, it's just a generic "give this object a pseudo-random-looking
but stable seconds value in `[0, 60)`" trick. Worth flagging as a rough edge:
`std::hash<std::string>` is not guaranteed to be stable across different
standard library implementations or even different program runs in general
C++ — but this codebase evidently accepts that tradeoff (reproducibility
*within* a build/platform, not necessarily across every possible compiler),
since the actual guarantee needed is just "same map, same run environment,
same offsets."

**Usage:** Called once per signalized junction, by `MapLoader`, using that
junction's id (see [`learn/08-supporting-systems.md`](08-supporting-systems.md)
and [`learn/02-core-graph-model.md`](02-core-graph-model.md) for
`Intersection`'s id). It's also called directly in the test suite (see
below) to verify the reproducibility and desync properties in isolation,
without going through a full map load.

---

### Lines 32–37: `state_at` — cycle and slot setup

```cpp
TrafficLight::State TrafficLight::state_at(int group, double sim_time) const {
    if (group < 0 || group >= groups) return Red;  // unassigned: fail closed

    const double slot = green_dur + amber_dur + all_red_dur;
    const double cycle = slot * groups;
    if (cycle <= 0.0) return Green;  // misconfigured: do not trap anyone
```

**Meaning:** First, the fail-closed guard from the header's doc comment:
any `group` outside `[0, groups)` returns `Red` immediately, short-circuiting
before any of the timing math runs. `slot` is one phase group's full
allotment of time — green plus amber plus the all-red clearance — and
`cycle` is the total time for *all* groups to take their turn once
(`slot * groups`). A second guard: if `cycle` computes to `0.0` or less
(only reachable if every one of `green_dur`/`amber_dur`/`all_red_dur` is
somehow `0.0` despite the constructor's floor of `1.0` on `green_dur` — so
in practice this branch guards against a state that shouldn't be reachable
through the public constructors, but would be reachable if this class ever
grew a way to set `green_dur` directly to `0.0` without going through the
clamp), returns `Green` unconditionally rather than dividing by a
zero-or-negative `cycle` a few lines later.

**Why:** Two different failure philosophies for two different problems.
Unassigned group → `Red` (fail closed, described above). Degenerate/zero
cycle → `Green` (fail open) — the inline comment explains why: "do not trap
anyone." If the cycle length is broken, defaulting to `Red` would mean every
approach at this junction is permanently stopped with no future green ever
computed — an unrecoverable gridlock caused by a config bug, not real
congestion. Defaulting to `Green` instead means a misconfigured light fails
safe in the sense of "traffic keeps moving," at the cost of not actually
regulating that junction — the lesser of two problems.

**Usage:** This is the setup shared (nearly verbatim) with `remaining_at`
below — notice the duplication; `slot`/`cycle` are computed independently in
both functions rather than factored into a shared private helper. That's a
minor missed-reuse opportunity, not a correctness issue, since both copies
compute the identical formula from the same immutable members.

---

### Lines 39–45: `state_at` — locating the active group

```cpp
    double t = std::fmod(sim_time + phase_offset, cycle);
    if (t < 0.0) t += cycle;

    // Which group owns the cycle right now. min() guards the case where fmod
    // returns exactly `cycle` through rounding.
    const int active = std::min(groups - 1, static_cast<int>(t / slot));
    if (active != group) return Red;
```

**Meaning:** `sim_time + phase_offset` shifts the raw simulation clock by
this junction's per-instance offset. `std::fmod(x, cycle)` is
floating-point remainder — it maps that shifted time down into `[-cycle,
cycle]` (technically it can be negative if `sim_time + phase_offset` is
negative), giving `t`, the position within one repetition of the cycle. The
`if (t < 0.0) t += cycle;` line normalizes a negative remainder back into
`[0, cycle)` — `std::fmod` follows the sign of its first argument, so a
negative input produces a negative result, which this line corrects.
`static_cast<int>(t / slot)` divides `t` by one group's slot length to find
which 0-indexed slot `t` falls into, truncating toward zero; `std::min(groups
- 1, ...)` clamps that index down to `groups - 1` as a defensive guard
against floating-point rounding pushing `t` to exactly `cycle` (which would
otherwise compute an out-of-range slot index of `groups`). If the computed
`active` group isn't the `group` the caller asked about, that group is red
— its turn hasn't come.

**Why:** This block *is* the "opposing approaches must be able to disagree"
guarantee from the class comment, implemented: at any instant, exactly one
`active` group index is "on," and every other group short-circuits to `Red`
here. The `t < 0.0` correction matters because `sim_time` is a `double`
representing seconds since some simulation epoch — it's always meant to be
non-negative in this codebase, but `phase_offset` is added *before* the
`fmod`, and defensive code doesn't assume the sum stays non-negative just
because both operands individually are.

**Usage:** This is the heart of what makes a four-way intersection behave
correctly — the [`tests/unit_tests/test_signals.cpp`](../tests/unit_tests/test_signals.cpp)
test "opposing approaches never show green at the same moment" (summarized
below) exists specifically to pin this logic down.

---

### Lines 47–50: `state_at` — colour within the active slot

```cpp
    const double u = t - active * slot;
    if (u < green_dur) return Green;
    if (u < green_dur + amber_dur) return Amber;
    return Red;  // clearance interval: everyone is red
}
```

**Meaning:** `u` is `t` relative to the start of the active group's own
slot — i.e., "how far into *this group's* turn are we," in `[0, slot)`.
Three-way comparison against the cumulative thresholds: less than
`green_dur` → `Green`; between `green_dur` and `green_dur + amber_dur` →
`Amber`; anything else (i.e. the remainder of the slot, the all-red
clearance interval) → `Red`.

**Why:** This is a straightforward "which bucket does `u` fall into" ladder
— the kind of code that's easy to get subtly wrong with off-by-one boundary
conditions, which is presumably why the test suite checks the cycle
exhaustively at `0.25`-second steps rather than at a handful of hand-picked
instants. The final `return Red` covers the all-red clearance interval, and
its comment says exactly what that interval is for: "everyone is red" —
this is the gap after one group's amber ends and before the *next* group's
green begins, during which every approach at the junction is red
simultaneously, giving vehicles already inside the junction time to clear
before the next stream is released. That's the "all-red clearance" concept
named in the header's class comment.

**Usage:** This return value flows straight back through
`Intersection::signal_state_for` to whoever asked — `Vehicle`'s
stop-line check or the frontend's colour display.

---

### Lines 53–70: `remaining_at` — same setup, different question

```cpp
double TrafficLight::remaining_at(int group, double sim_time) const {
    if (group < 0 || group >= groups) return 0.0;

    const double slot = green_dur + amber_dur + all_red_dur;
    const double cycle = slot * groups;
    if (cycle <= 0.0) return 0.0;

    double t = std::fmod(sim_time + phase_offset, cycle);
    if (t < 0.0) t += cycle;

    const double mine = group * slot;          // start of this group's slot
    const double u = t - mine;
    if (u < 0.0) return -u;                    // waiting for our slot to open
    if (u < green_dur) return green_dur - u;
    if (u < green_dur + amber_dur) return green_dur + amber_dur - u;
    if (u < slot) return slot - u;
    return cycle - u + mine;                   // our slot already passed
}
```

**Meaning:** The first eight lines duplicate `state_at`'s setup exactly,
except the two guard clauses return `0.0` instead of `Red`/`Green` (matching
`remaining_at`'s `double` return type — "0 seconds remaining" is the
sensible degenerate answer for an unassigned group or a broken cycle,
mirroring but not identical to `state_at`'s choices). `mine` is where
*this* group's slot starts within the cycle (`group * slot`, not `active *
slot` — this function doesn't need to find which group is active, only
where the *queried* group's slot sits). `u` is `t` relative to that start,
and it can now legitimately be **negative** — unlike `state_at`'s `u`, which
was always relative to whichever group was active. Four cases, in order:
`u < 0.0` means the queried group's slot hasn't started yet this cycle, so
the answer is `-u` (seconds until it starts); `u < green_dur` means
currently green, answer is time left in green; `u < green_dur + amber_dur`
means currently amber, same pattern; `u < slot` means in the all-red
clearance, time left until the slot ends; and the fallback,
`cycle - u + mine`, covers the case where this group's slot for *this*
repetition of the cycle has already fully passed, so the answer wraps
around to that group's start in the *next* repetition of the cycle.

**Why:** The reason this can't just reuse `state_at`'s `active`-based
positioning is the question being asked is different: `state_at` only ever
needs to know the colour of whichever group happens to be active *right
now*; `remaining_at` needs a countdown for a *specific* group regardless of
whether that group is currently active, upcoming later this cycle, or its
turn already passed this time around. That's why `mine` is anchored to the
queried `group`, not to whatever `active` currently is, and why there are
four cases here instead of three — "not yet my turn" is a distinct state
`state_at` never needs to represent, because from `state_at`'s point of view
a not-yet-active group is simply red, full stop, with no need to say *how*
not-yet.

**Usage:** Reached the same way as `state_at` — through
`Intersection::remaining_at`, the one-line forwarder — feeding the
frontend's countdown display named in the header's doc comment.

---

### Lines 72–77: `set_green_duration` — the live mutation

```cpp
void TrafficLight::set_green_duration(double time) {
    // A light-failure event takes effect immediately. With a stateless cycle
    // that is automatic: the next state_at() already uses the new duration,
    // so there is no current phase left over to cut short.
    green_dur = std::max(1.0, time);
}
```

**Meaning:** Assigns `green_dur`, applying the same `std::max(1.0, ...)`
floor the constructor uses, so this can't be used to sneak `green_dur` below
the one-second minimum either.

**Why:** The comment is the payoff of the entire "pure function of time"
design established at the top of the header. A stateful, ticked light would
need explicit logic here to handle "we're currently mid-green when this
event fires — do we cut the current phase short, let it finish, or restart
it?" This class needs none of that: because `state_at`/`remaining_at`
recompute everything from `sim_time` on every call, the instant `green_dur`
changes, the *next* call to either function reflects the new value with no
separate "apply immediately" logic required. This is the concrete payoff of
the architectural choice flagged at the very top of this module — it's
worth noticing how much incidental complexity (phase-transition bookkeeping,
partial-phase interpolation) this design sidesteps entirely by not storing
state.

**Usage:** `TrafficLightEvent::apply`/`::expire`, as covered above, are the
only callers — see
[`learn/06-events-observer.md`](06-events-observer.md) for how a
`TrafficEvent` decides when to fire `apply` and `expire` against the graph.

---

## What the tests confirm

[`tests/unit_tests/test_signals.cpp`](../tests/unit_tests/test_signals.cpp)
(target `test_signals_unit` in `CMakeLists.txt`) exercises `TrafficLight`
both directly and through `Intersection`/`TrafficGraph`'s signal-assignment
machinery (the latter belongs to
[`learn/02-core-graph-model.md`](02-core-graph-model.md), but several cases
are worth knowing about here since they pin down `TrafficLight`'s own
contract):

- **Phase-group separation.** A four-way crossing splits its four approaches
  into exactly two groups, with the east-west pair sharing one group and the
  north-south pair the other — confirming `signal_group` assignment (a
  `Graph`/`Intersection` concern) feeds correctly into `state_at`'s grouping.
- **No simultaneous green across groups.** Sampled every 0.25 s across two
  full minutes, the east-west and north-south groups are never both green at
  once, and neither is starved — this is the block-by-block walk through
  `state_at`'s "which group is active" logic, verified exhaustively rather
  than at a few hand-picked instants (matching the concern raised above
  about off-by-one boundaries in that ladder of comparisons).
- **No drift.** Sampling one full cycle and re-sampling at `+cycle` and
  `+5*cycle` produces identical colours every time — the direct test of the
  "pure function, cannot drift" claim from the header comment.
- **Desynchronized junctions.** Two separate junctions signalized in the
  same graph get different `phase_offset` values — confirms
  `set_phase_offset_from` isn't accidentally deterministic-to-a-constant.
- **Reproducible offsets.** Calling `set_phase_offset_from("junction-42")`
  on two separate `TrafficLight` instances yields the same offset; a
  different id string yields a (very likely) different one — the direct
  test of `set_phase_offset_from`'s reproducibility guarantee, independent
  of any map loading.
- **Fail-closed for unassigned groups.** Querying `state_at`/
  `signal_state_for` with `group = -1` or `group = 99` returns `Red` across
  a full sampling sweep — the direct test of the header's "fails closed"
  guarantee for out-of-range group indices.
- **Designalizing is fail-open.** Once a junction's light is removed
  (`designalize`), every approach reports `Green` regardless of
  `sim_time` — this is `Intersection::signal_state_for`'s own `if
  (!light_system) return TrafficLight::Green;` guard (in
  `Intersection.cpp`, not `TrafficLight.cpp` itself), included here because
  it completes the picture of what "no light" versus "unassigned group at a
  lit junction" each mean: the former is deliberately open, the latter is
  deliberately closed.

Note several of these tests exercise `signalize`/`assign_signal_groups`/
`stop_line_position`, which live on `Graph`/`Road`/`Intersection`, not on
`TrafficLight` itself — those are covered properly in
[`learn/02-core-graph-model.md`](02-core-graph-model.md).

## Check yourself

- Why does `state_at` fail closed (`Red`) for an out-of-range `group`, while
  its `cycle <= 0.0` guard fails open (`Green`)? What real-world failure is
  each guard protecting against?
- `remaining_at` has four branches for `u` where `state_at` has three. What
  extra case does `remaining_at` need to represent that `state_at` doesn't,
  and why does `state_at` not need it?
- Why is `phase_offset` derived from the junction's id via `std::hash`
  rather than assigned by load order or drawn randomly at construction? What
  would break if it were random?
- `set_green_duration` needs no logic to handle "we're mid-phase when this
  is called." What property of the class as a whole makes that true, and
  what would a ticked, stateful light need to do instead in the same
  situation?
