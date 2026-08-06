# Module 02 — Core Graph Model

**Files covered:**
[`cpp_backend/include/core/Graph.hpp`](../cpp_backend/include/core/Graph.hpp) /
[`cpp_backend/src/core/Graph.cpp`](../cpp_backend/src/core/Graph.cpp),
[`cpp_backend/include/core/Intersection.hpp`](../cpp_backend/include/core/Intersection.hpp) /
[`cpp_backend/src/core/Intersection.cpp`](../cpp_backend/src/core/Intersection.cpp),
[`cpp_backend/include/core/Road.hpp`](../cpp_backend/include/core/Road.hpp) /
[`cpp_backend/src/core/Road.cpp`](../cpp_backend/src/core/Road.cpp),
[`cpp_backend/include/core/Building.hpp`](../cpp_backend/include/core/Building.hpp) /
[`cpp_backend/src/core/Building.cpp`](../cpp_backend/src/core/Building.cpp),
[`cpp_backend/include/core/Zone.hpp`](../cpp_backend/include/core/Zone.hpp) — **no `Zone.cpp` exists**;
confirmed by listing `cpp_backend/src/core/` (only `Building.cpp`, `Graph.cpp`, `Intersection.cpp`,
`Road.cpp`, `TrafficLight.cpp` are there) and by `CMakeLists.txt`'s `BACKEND_SOURCES` list, which
never mentions a `Zone.cpp` (see [`learn/01-orientation.md`](01-orientation.md)). `Zone` is a plain
aggregate `struct` with no member functions to define out of line, so there is nothing a `.cpp`
would contain.

This is the map itself: a directed graph of `Intersection` nodes and `Road` edges, plus two
passenger types — `Building` (things anchored to the graph) and `Zone` (background decoration with
no graph relationship at all). Everything downstream — routing (`learn/04-routing.md`),
vehicles, the tick loop — walks this graph; nothing here depends on them. `TrafficLight` lives on
`Intersection` (as `light_system`) but its internals are covered in
[`learn/03-traffic-lights.md`](03-traffic-lights.md); this module treats it as an opaque owned
object.

---

## `cpp_backend/include/core/Graph.hpp`

### Lines 1–11: includes and forward declarations

```cpp
#pragma once
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include "core/Zone.hpp"

// Use forward declarations instead of #include to speed up builds and avoid circular dependencies
class Intersection;
class Road;
class Building;
```

**Meaning:** `#pragma once` is a compiler-specific (but universally supported) include guard —
this header's contents are pasted into a translation unit at most once, no matter how many times
it's `#include`d transitively. `Zone.hpp` is included in full because `TrafficGraph` stores
`std::vector<Zone>` by value (below) — a `vector<Zone>` member needs `Zone`'s complete definition
to know its size and how to construct/destroy elements. `Intersection`, `Road`, and `Building` are
only forward-declared: the class name is announced without its definition, which is legal because
this header only ever refers to them through pointers (`Intersection*`) or as the template argument
of `unique_ptr<T>`, neither of which needs a complete type at *declaration* time.

**Why:** the inline comment states the rationale directly — forward declarations avoid a circular
`#include` chain (`Road.hpp` needs `Intersection`, `Intersection` doesn't need `Road`'s full
definition either) and cut compile time, because any translation unit that only calls
`graph.get_edge(...)` and stores the `Road*` doesn't need `Road.hpp`'s full contents pulled in.
`Zone` gets the opposite treatment because `TrafficGraph` owns `Zone` objects by value, not by
pointer — there's no way to declare a `vector<Zone>` member with only a forward declaration.

**Usage:** This split is what forces `Graph.cpp` to `#include "core/Intersection.hpp"`,
`"core/Road.hpp"`, and `"core/Building.hpp"` itself (see below) — anyone who wants to actually call
into those types needs the real headers, but `Graph.hpp`'s own consumers don't pay that cost unless
they ask for it.

**Check yourself:**
- Why can `TrafficGraph` declare `std::unique_ptr<Intersection>` with only a forward declaration, but not `std::vector<Intersection>`?
- What would break if `Zone.hpp` were forward-declared instead of `#include`d here?

---

### Lines 13–22: the owning containers

```cpp
class TrafficGraph {
public:
    std::unordered_map<std::string, std::unique_ptr<Intersection>> intersections;
    std::unordered_map<std::string, std::unique_ptr<Road>> edges;
    std::unordered_map<std::string, std::unique_ptr<Building>> buildings;
    std::vector<Zone> zones; // Display-only background regions, loaded from the GeoJSON map
```

**Meaning:** Three hash maps, each keyed by a `std::string` id (an intersection name, a road id, a
building name) and each holding a `std::unique_ptr` to the actual object. `TrafficGraph` is the sole
owner: when an entry is erased from the map, or the map itself is destroyed, the pointed-to object
is deleted automatically — nobody else in the codebase is expected to `delete` an `Intersection`,
`Road`, or `Building`. `zones` is a plain `std::vector<Zone>` — value storage, not pointer storage,
because (per the earlier comment) `Zone` never needs identity-based lookup or a stable address the
way a `Road*` does; it's read sequentially for drawing.

**Why:** `unique_ptr` inside a map, rather than storing `Intersection` objects directly, keeps the
objects at a stable heap address even as the map itself rehashes and reshuffles its buckets — every
other piece of code holds raw `Intersection*`/`Road*` pointers into this graph (see
`Intersection::outgoing_edges`, a `vector<Road*>`, below), and those pointers would dangle the
instant a `std::unordered_map<std::string, Intersection>` (objects by value) rehashed and moved its
elements. The `TrafficGraph.hpp` comment spells out the corollary: because ownership is
`unique_ptr`, the compiler-generated destructor is correct with no manual delete loop (Rule of
Zero), and there's no way for a lookup to return a dangling pointer that still looks valid — a
missing key returns `nullptr` from `.find()`, never a stale pointer.

**Usage:** `MapLoader` (see `learn/08-supporting-systems.md`) populates these three maps and the
`zones` vector while reading a GeoJSON file. `Road::Road()`'s constructor (below) pushes `this` into
`source->outgoing_edges` and `target->incoming_edges` — those `vector<Road*>` members only remain
valid because the `Road` objects they point at live in `TrafficGraph::edges` and never move once
constructed.

**Usage — the "junction overlap" limitation:** nothing in this header enforces that two
`Intersection`s can't sit at (or near) the same `(x, y)` — the map is a flat, unordered
`unordered_map<std::string, unique_ptr<Intersection>>`, with no spatial index and no
insertion-time collision check. If two GeoJSON nodes happen to land at overlapping coordinates,
`TrafficGraph` will store both without complaint. This is consistent with the project's documented
"junction overlapping can happen" limitation (see the MASTER_TUTOR_PROMPT's Known Limitations, and
`learn/07-simulation-core.md` for where that surfaces in the tick loop) — the fix would have to live
either in `MapLoader` (reject/merge overlapping nodes at load time) or in a spatial check added
here, and neither exists today.

**Check yourself:**
- Why store `unique_ptr<Intersection>` inside the map instead of `Intersection` by value?
- What guarantees that a `Road*` obtained from `TrafficGraph::get_edge()` stays valid for the lifetime of the graph?
- Given that `intersections` is an `unordered_map` with no spatial check, what would it take to detect two overlapping junctions at load time?

---

### Lines 24–46: constructors, destructor, and the copy/move story

```cpp
TrafficGraph();
~TrafficGraph();

TrafficGraph(const TrafficGraph&) = delete;
TrafficGraph& operator=(const TrafficGraph&) = delete;
TrafficGraph(TrafficGraph&&);
TrafficGraph& operator=(TrafficGraph&&);
```

**Meaning:** Four special member functions are *declared* here but none is defined inline — the
comment above them (lines 24–36 in the file) explains why: even though `TrafficGraph()` and
`~TrafficGraph()` would look trivial enough to default inline, the compiler still needs to reason
about whether `TrafficGraph()` is `noexcept`, which requires checking whether
`unordered_map<string, unique_ptr<Building>>`'s default constructor is `noexcept` — and at least on
libstdc++, that check instantiates the map's destructor as a side effect. Instantiating that
destructor needs `Building` to be a complete type, which it isn't yet in this header (only forward
declared). So even the *constructor* — not just the destructor — needs `Intersection`, `Road`, and
`Building` to be complete, and that completeness is only available in `Graph.cpp`, after its own
`#include`s. Copy construction and copy assignment are explicitly `= delete`d — the class is
non-copyable. Move construction and move assignment are declared (defaulted out-of-line in the
`.cpp`, again for the same completeness reason) — the class is movable.

**Why:** Copying a `TrafficGraph` would either shallow-copy the `unique_ptr`s (impossible — the
compiler simply won't generate a copy constructor for a type containing `unique_ptr` members
without a user override, since `unique_ptr` itself has no copy constructor) or require deep-cloning
every `Intersection`, `Road`, and `Building` in the graph, which nothing in the codebase ever needs.
`= delete` makes an accidental `TrafficGraph copy = original;` a compile error instead of silently
falling back to some unintended behavior. Move is different: swapping out the internal
`unordered_map`/`vector` handles is cheap (no deep copy, no reallocation of the owned objects) and
is exactly what's needed when, say, `MapLoader::load()` builds a `TrafficGraph` on the stack and
hands it back by value, or `import_map()` swaps a freshly-loaded graph in to replace the live one.

**Usage:** Anywhere the simulator loads or reloads a map, it's moving a `TrafficGraph`, not copying
one — see `learn/08-supporting-systems.md` for `MapLoader::load()`/`import_map()`, which the
`Graph.hpp` comment names explicitly as the callers of the move constructor/assignment.

**Check yourself:**
- Why does even the *default constructor* of `TrafficGraph` need `Intersection`/`Road`/`Building` to be complete types, not just the destructor?
- What error would you get if you tried `TrafficGraph g2 = g1;` given the deleted copy constructor?

---

### Lines 48–61: `create_directional_road`, `get_edge`, `reindex`

```cpp
void create_directional_road(const std::string& edge_id, Intersection* source_node, Intersection* target_node, int lanes, double speed_limit, bool is_one_way);
Road* get_edge(const std::string& edge_name) const;
void reindex();
```

```cpp
void TrafficGraph::create_directional_road(const std::string& edge_id, Intersection* source_node, Intersection* target_node,
                                           int lanes, double speed_limit, bool is_one_way) {
    auto new_road = std::make_unique<Road>(edge_id, source_node, target_node, lanes, speed_limit);
    new_road->is_one_way = is_one_way;
    edges[edge_id] = std::move(new_road);
}

Road* TrafficGraph::get_edge(const std::string& edge_name) const {
    auto it = edges.find(edge_name);
    if (it != edges.end()) {
        return it->second.get();
    }
    return nullptr;
}
```

**Meaning:** `create_directional_road` builds a `Road` with `std::make_unique` (constructs the
`Road` and wraps it in a `unique_ptr` in one call, avoiding a naked `new`), sets `is_one_way`
*after* construction rather than through the constructor's own `is_one_way` parameter (the `Road`
constructor takes an `is_one_way` argument too — this looks like a slightly redundant belt-and-
suspenders assignment, since the constructor's parameter list already includes it; the
post-construction set is harmless but effectively a duplicate write of the same value), then
`std::move`s the `unique_ptr` into the `edges` map, transferring ownership — `new_road` is left
empty after this line. `get_edge` is a `const` lookup: `edges.find()` returns an iterator, and if it
isn't `end()` the raw `Road*` is extracted via `.get()` (which does *not* transfer ownership — the
map still owns the `Road`). A missing key returns `nullptr`.

**Why:** `make_unique` over `new` + manual wrap is the standard modern-C++ idiom for exception
safety — there's no window where a raw pointer exists unowned. Returning a raw `Road*` from
`get_edge` (rather than a reference to the `unique_ptr`, or the `unique_ptr` itself) is deliberate:
callers need to check for "not found" (`nullptr`), and returning a raw non-owning pointer makes the
non-ownership explicit — nobody who calls `get_edge` should ever `delete` the result.

**Usage:** `get_edge` is the function every other subsystem uses to resolve a road id string back
to a live `Road*` — routing, `TrafficEvent` handlers, the map-editing API, and the WebSocket message
handlers all funnel through it. `create_directional_road` is a lower-level builder; `MapLoader` uses
it (or constructs `Road` directly — see `learn/08-supporting-systems.md`) while reading map data.

```cpp
void TrafficGraph::reindex() {
    uint32_t node_idx = 0;
    for (auto& [id, node] : intersections) {
        if (node) node->index = node_idx++;
    }

    uint32_t edge_idx = 0;
    uint32_t base_idx = 0;
    std::unordered_map<std::string, uint32_t> base_name_to_id;
    for (auto& [id, road] : edges) {
        if (!road) continue;
        road->index = edge_idx++;
        const bool is_reverse = id.size() > 2 && id.compare(id.size() - 2, 2, "_r") == 0;
        std::string base_id = is_reverse ? id.substr(0, id.size() - 2) : id;
        auto [it, inserted] = base_name_to_id.try_emplace(base_id, base_idx);
        if (inserted) base_idx++;
        road->base_index = it->second;
        road->reverse_twin = is_reverse;
    }
}
```

**Meaning:** `reindex()` walks `intersections` and assigns each a dense `uint32_t index` from `0` up
to `intersections.size() - 1`, in whatever order the `unordered_map` happens to iterate (hash-bucket
order, not insertion order — this matters, see below). It does the analogous thing for `edges`, but
with an extra wrinkle: a two-way street is stored as two separate `Road` objects sharing a "base"
name — the forward one named e.g. `"r12"` and the reverse one named `"r12_r"`. The loop strips a
trailing `"_r"` suffix (checked with `id.compare(id.size() - 2, 2, "_r") == 0`, a substring compare
rather than allocating a new string just to compare) to compute `base_id`, then uses
`try_emplace` on `base_name_to_id` to either look up an existing `base_index` for that base name or
allocate a fresh one (`try_emplace` inserts only if the key is absent, and its return value tells
you which case happened via the `inserted` bool). `road->reverse_twin` records whether *this*
`Road` object was the `"_r"` half.

**Why:** the header comment explains the payoff directly: `Intersection::index` and `Road::index`
back **O(1)** array-indexed lookups in `TrafficSimulator::solve_junctions()` (see
`learn/07-simulation-core.md`) instead of hashing a string every time the tick loop needs to find a
node's or road's slot in some parallel array. `base_index` and `reverse_twin` exist so that code
which cares about "the street", not "which direction of it" (deciding, say, that both directions of
the same physical street should be drawn or grouped together) doesn't need to string-compare `id`
against `"_r"` on a hot path — that comparison is done once, here, and cached as a bare `bool` and
`uint32_t` any caller can read for free.

**Usage:** The comment is explicit and worth repeating verbatim: *"call this after ANY edit that
adds or removes an intersection or road... live map editing must call it again, or new elements keep
the default index 0 and silently alias whatever already owns that slot."* That's a real footgun —
`reindex()` is not automatically triggered by mutating `intersections`/`edges`; it's the caller's
responsibility. `MapLoader` calls it once after the initial load; `MapEditor` (see
`learn/08-supporting-systems.md`) must call it again after any live edit, or a newly added
`Intersection`/`Road` will sit at index `0`, aliasing whatever object already holds that array slot
in `TrafficSimulator`'s indexed structures — a correctness bug with no compiler warning to catch it.

**Check yourself:**
- Why does `get_edge` return a raw `Road*` instead of `const std::unique_ptr<Road>&`?
- What happens if a caller adds a new `Road` to `edges` directly (bypassing `create_directional_road`) and forgets to call `reindex()`?
- Why is `unordered_map` iteration order (not insertion order) safe to use for assigning `index`, given that `reindex()` is meant to be re-run after edits?

---

### Lines 63–91: signal assignment — `assign_signal_groups`, `signalize`, `designalize`

```cpp
int assign_signal_groups(const std::string& node_id);
bool signalize(const std::string& node_id, double green = 25.0);
void designalize(const std::string& node_id);
```

**Meaning:** Three related entry points for turning a plain junction into (or back out of) a
signalized one. `assign_signal_groups` does the geometric work of clustering the junction's incoming
roads into phase groups and returns how many distinct groups it found (documented precisely as `0`,
`1`, or `2` — see the header's own comment). `signalize` is `assign_signal_groups` plus actually
attaching a `TrafficLight`; it takes a default `green = 25.0` (seconds) so most callers don't have
to think about the timing. `designalize` removes the light and resets the approach roads' groups.

**Why:** the header comment draws a sharp distinction between two callers with different tolerance
for a false-negative: *"Callers that GUESS a signal should insist on 2. Callers loading a signal a
map file states explicitly should accept 1, or saving and reloading a map would silently drop the
light the author put there."* This is a real design tension — a junction where every approach lies
on one axis has nothing to arbitrate (there's no crossing stream), so a heuristic that's deciding
*whether* to add a signal should refuse. But if the *map file itself* already says "this junction has
a light" (e.g. an OSM `highway=traffic_signals` tag that MapLoader read), refusing to honor it just
because the geometry looks degenerate would silently corrupt round-tripped map data. Splitting
"compute the groups" from "attach a light" is what lets both callers share the geometry logic while
disagreeing on the accept/reject threshold.

```cpp
int TrafficGraph::assign_signal_groups(const std::string& node_id) {
    auto it = intersections.find(node_id);
    if (it == intersections.end()) return 0;
    Intersection* node = it->second.get();
    if (node->is_roundabout || node->zone == "waypoint") return 0;

    const std::vector<Road*>& in = node->incoming_edges;
    if (in.empty()) return 0;
```

**Meaning:** Look up the node by id; bail out with `0` (the "refused" sentinel) if it doesn't
exist, if it's a roundabout, if its `zone` string is `"waypoint"`, or if it has no incoming roads at
all. `in` is a `const&` alias to the node's `incoming_edges` — no copy of the vector.

**Why:** roundabouts and waypoints are structurally the wrong shape for a two-phase signal — a
roundabout's whole point is that traffic yields rather than stops in turns, and a waypoint (likely a
synthetic node used for geometry/routing, not a real junction) has no real "crossing streams" to
arbitrate. Refusing early avoids doing the trigonometry below for cases where the answer is
meaningless.

**Usage:** This is the guard that `signalize()` relies on to keep guessed signals sane — see below.

```cpp
    constexpr double kPi = 3.14159265358979323846;
    std::vector<int> group(in.size(), 0);
    double seed = 0.0;
    bool has0 = false, has1 = false;
    for (std::size_t i = 0; i < in.size(); ++i) {
        const Intersection* from = in[i]->source;
        if (!from) return 0;
        const double bearing = std::atan2(node->y - from->y, node->x - from->x);
        if (i == 0) seed = bearing;
        const double axis_diff = std::fabs(std::remainder(bearing - seed, kPi));
        group[i] = axis_diff <= kPi / 4.0 ? 0 : 1;
        (group[i] == 0 ? has0 : has1) = true;
    }
```

**Meaning:** For each incoming road, compute the *bearing* — the angle of travel as a vehicle
arrives at `node`, using `atan2(node->y - from->y, node->x - from->x)` (the direction from the
approach's source intersection toward this one). The first approach's bearing (`i == 0`) becomes the
reference `seed`. `std::remainder(bearing - seed, kPi)` computes how far `bearing` is from `seed`,
wrapped modulo π (not 2π) — folding by π is what makes a road's two directions (arriving
northbound vs. southbound on the same physical street) compare as the *same* axis, since they're
180° = π radians apart and `remainder` wraps that back near zero. `axis_diff <= kPi/4.0` (a 45°
threshold) decides whether this approach belongs to the seed's axis (`group 0`) or the other one
(`group 1`). `has0`/`has1` track whether any approach landed in each bucket, using the somewhat
terse `(group[i] == 0 ? has0 : has1) = true;` — a conditional expression used as an lvalue to pick
which bool to set, avoiding an `if/else` for a single assignment.

**Why:** the comment above the loop explains the intent precisely: two approaches on the same
street should share a group and go green together, while the physically crossing street waits. This
is a purely geometric heuristic — it has no notion of lane count, road class, or actual OSM tagging;
it only looks at *the angle vehicles arrive from*. That's a reasonable approximation for a simple
four-way or T intersection, but it's fragile for anything more irregular (a five-way junction, or two
roads meeting at a shallow angle near the 45° boundary could get misclassified into the wrong axis).
If `!from` (an incoming road with no source intersection — shouldn't normally happen but is checked
defensively), the function bails with `0` rather than computing garbage geometry.

**Usage:** This is the core of "guessing" a signal from geometry alone, used by `signalize()`. It's
also worth noting as an example of the kind of geometric reasoning that, elsewhere in this codebase,
underlies the "junction overlapping can happen" limitation: bearing-based classification assumes
`from->x/y` and `node->x/y` are meaningfully distinct and non-degenerate — two intersections placed
at (or very near) the same coordinates would produce an undefined or wildly unstable `atan2`, though
this function itself doesn't currently guard against that case.

```cpp
    const bool crossing = has0 && has1;
    for (std::size_t i = 0; i < in.size(); ++i) {
        in[i]->signal_group = crossing ? group[i] : 0;
        in[i]->signal_stop_from_end = std::min(8.0, in[i]->length * 0.4);
    }
    return crossing ? 2 : 1;
}
```

**Meaning:** `crossing` is true only if both buckets got at least one approach. The second loop
writes the actual result onto each `Road`: `signal_group` is either the computed bucket (`0`/`1`) if
there's real crossing traffic, or forced to `0` for everyone if there's only one axis (so the
junction still gets a coherent single-phase cycle rather than half its approaches left with an
unset/stale group). `signal_stop_from_end` is set to `min(8.0, length * 0.4)` — normally 8 meters
back from the junction, but capped at 40% of the road's own length on short blocks, with the comment
explaining why: a fixed 8 m stop line on a short OSM-derived block would sit *before the road even
starts*, making a vehicle brake for a line that's behind it. Returns `2` (real crossing, a signal is
warranted) or `1` (one axis only, a map-stated signal should still be honored).

**Why:** already covered above — this is where the 0/1/2 contract described in the header comment
is actually produced.

**Usage:** `signalize()` (below) checks this return value directly.

```cpp
bool TrafficGraph::signalize(const std::string& node_id, double green) {
    if (assign_signal_groups(node_id) < 2) return false;
    Intersection* node = intersections[node_id].get();
    node->light_system = std::make_unique<TrafficLight>(green, 3.0, 2.0, 2);
    node->light_system->set_phase_offset_from(node->name);
    return true;
}
```

**Meaning:** Calls `assign_signal_groups` and refuses (`return false`) unless it reported a real
crossing (`2`). Otherwise looks the node up again — this time with `operator[]` rather than `.find()`
(safe here only because `assign_signal_groups` having returned `2` already proved the key exists) —
and constructs a `TrafficLight` with `green` seconds of green, then two more numeric arguments
(`3.0`, `2.0`) and a `2` whose meaning is defined by `TrafficLight`'s own constructor, not explained
here (see `learn/03-traffic-lights.md`). `set_phase_offset_from(node->name)` presumably derives a
deterministic phase offset from the node's own name/id, so that adjacent signals aren't all
switching in lockstep by coincidence — again, its internals belong to the `TrafficLight` module.

**Why:** the "insist on 2" rule from the header comment is enforced here, in code, not just in
documentation — a heuristic caller (something guessing "this junction looks like it needs a light")
cannot accidentally signalize a junction with no real crossing streams.

**Usage:** This is presumably what `MapEditor`'s "add a signal" UI action calls, and possibly what
`MapLoader` calls for junctions the source map didn't explicitly tag but that look signal-worthy —
confirming that split precisely is a question for `learn/08-supporting-systems.md`.

```cpp
void TrafficGraph::designalize(const std::string& node_id) {
    auto it = intersections.find(node_id);
    if (it == intersections.end()) return;
    Intersection* node = it->second.get();
    for (Road* r : node->incoming_edges) r->signal_group = -1;
    node->light_system.reset();
}
```

**Meaning:** Looks the node up safely with `.find()` this time (no assumption the key exists), and
if found, resets every incoming road's `signal_group` back to `-1` ("no signal on this approach" —
see `Road.hpp`) and calls `.reset()` on the `unique_ptr<TrafficLight>`, which destroys the
`TrafficLight` object and sets the pointer to `nullptr`.

**Why:** clearing `signal_group` on every approach is necessary, not optional — if only
`light_system` were reset, the roads would still carry stale phase-group numbers with no light to
interpret them, and (per `Intersection::is_signalized()`, below) the *light* is the single source of
truth for "is this junction signalized," so leaving stale groups around wouldn't cause a visible bug
today, but it's still leftover state that a future feature reading `signal_group` directly (without
going through `is_signalized()`) could misinterpret.

**Usage:** The inverse of `signalize()`; likely wired to a "remove signal" UI action in `MapEditor`.

**Check yourself:**
- Why does `assign_signal_groups` fold bearings modulo π rather than 2π when deciding which axis an approach belongs to?
- Walk through what happens if `signalize()` is called on a junction where `assign_signal_groups` returns `1` — what does the caller see, and is that the right behavior for a heuristic "should I add a light here?" feature versus a "the map file said so" loader?
- Why does `designalize()` use `.find()` while `signalize()` uses `operator[]` right after a check that guarantees the key exists — are both safe, and why?

---

## `cpp_backend/include/core/Intersection.hpp` / `cpp_backend/src/core/Intersection.cpp`

### Lines 9–26: fields and the constructor

```cpp
class Intersection {
public:
    std::string name;         // Intersection id, used as the key in TrafficGraph
    uint32_t index = 0;       // Fast zero-overhead integer ID
    std::string display_name; // Human-readable label, e.g. "Hillside & 1st"
    std::string zone;         // Region the node sits in ("", "downtown", "roundabout", ...)
    double x;
    double y;
    bool is_building;
    bool is_roundabout;
    double radius;
    std::vector<Road*> outgoing_edges;
    std::vector<Road*> incoming_edges;

    std::unique_ptr<TrafficLight> light_system;

    Intersection(std::string name, double x, double y);
    ~Intersection() = default;
```

**Meaning:** Every field here is public data, not accessed through getters — `name` doubles as the
key `TrafficGraph::intersections` uses to store this object, `index` is the dense id
`TrafficGraph::reindex()` assigns, `display_name` is a separate human-facing label, `zone` is a
free-form string ("", "downtown", "roundabout", "waypoint", ... — `assign_signal_groups` above
checks it against `"waypoint"` by exact string match, so this is effectively a soft enum). `x`/`y`
are world coordinates. `is_building` marks a node that's really a building's anchor point rather
than a real junction (see `is_intersection()` below). `is_roundabout` and `radius` describe
roundabout geometry. `outgoing_edges`/`incoming_edges` are `vector<Road*>` — non-owning; the actual
`Road` objects are owned by `TrafficGraph::edges`, and every `Road` constructor pushes itself into
its endpoints' vectors (see `Road.cpp` below). `light_system` is the one owning pointer on this
class — a `unique_ptr<TrafficLight>`, `nullptr` when the junction is unsignalized. The constructor
only takes `name`, `x`, `y` — everything else defaults via the member-initializer list in the `.cpp`.
`~Intersection() = default` — the compiler-generated destructor is fine because `unique_ptr` and
`vector` clean up after themselves.

**Why:** public data here is a deliberate, consistent choice across this module (see the same
reasoning spelled out explicitly on `Road`'s "topology" fields) — these are fields read constantly
on hot paths (pathfinding walks `outgoing_edges`; the renderer reads `x`/`y`/`is_roundabout`/
`radius` every frame) with no invariant a getter would meaningfully protect. Contrast this with
`Road`'s *private* fields (`lanes_`, `speed_limit_`, etc.) further down, which *do* have invariants
(a valid range, "closed roads are impassable") enforced through setters — the pattern in this
codebase is: public data by default, private + accessor only when there's a real invariant worth
guarding.

**Usage:** `TrafficGraph::intersections` owns `Intersection` objects behind `unique_ptr`; every other
piece of code — routing, the tick loop, the renderer, `MapEditor` — holds raw `Intersection*` into
that map and reads these fields directly.

**Check yourself:**
- Why are `outgoing_edges`/`incoming_edges` stored as `vector<Road*>` (non-owning) rather than `vector<unique_ptr<Road>>`?
- What would go wrong if `Intersection` were copyable and someone wrote `Intersection copy = *node;`? (Hint: look at what `light_system` is.)

---

### Lines 28–42: the deleted copy, and `is_signalized()` as derived state

```cpp
    Intersection(const Intersection&) = delete;
    Intersection& operator=(const Intersection&) = delete;

    bool is_intersection() const;

    bool is_signalized() const { return light_system != nullptr; }
```

**Meaning:** Copy construction and copy assignment are explicitly deleted. `is_intersection()` is
declared here, defined in the `.cpp`. `is_signalized()` is defined *inline*, right in the header —
a one-line function body — returning whether `light_system` is non-null.

**Why:** Copy is deleted for the same reason as `TrafficGraph`'s: `light_system` is a `unique_ptr`,
so a default copy constructor wouldn't even compile, and the comment adds the real point — "two
intersections should never share one light's ownership." The `is_signalized()` comment is worth
quoting because it documents a real bug class the author explicitly avoided: *"Derived, not a
separately-stored flag... The previous code stored both and defensively checked
`is_signalized && light_system` in `signal_state_for()` — exactly the symptom of two things meant to
agree that were not actually guaranteed to."* This is a concrete example of eliminating a class of
bug by construction: there is no separate `bool is_signalized_` field that could ever disagree with
whether `light_system` is actually set, because the *function* computes the answer from the one
field that matters, every time, instead of caching a second copy of the same fact.

**Usage:** `is_signalized()` is presumably read by the renderer (to decide whether to draw a light)
and possibly by routing/UI logic; `signal_state_for()`/`signal_remaining_for()` (below) are the
actual behavior-affecting queries used by the tick loop.

```cpp
bool Intersection::is_intersection() const {
    return !is_building && incoming_edges.size() > 1;
}
```

**Meaning:** A node counts as a "real" intersection only if it's not a building anchor and has more
than one incoming road.

**Why:** this distinguishes a genuine junction (where multiple streams of traffic actually meet and
need arbitration) from a degenerate node — a dead end, a mid-block point, or a building's anchor
point — that happens to exist in the graph but isn't a place where routing/signal decisions matter.

**Usage:** Likely consulted by the "guess a signal" UI flow before even offering to call
`signalize()`, and by the renderer to decide how to draw a node (junction marker vs. plain vertex).

**Check yourself:**
- Why is `is_signalized()` defined inline in the header while `is_intersection()` is defined out of line in the `.cpp` — does that difference matter functionally?
- What bug did the author avoid by making `is_signalized()` derived rather than a stored flag? Can you think of an analogous pattern elsewhere in this module (hint: `Road::stop_line_position()` also derives its answer from `signal_group` rather than a separate flag)?

---

### Lines 44–47 (header) / full `.cpp`: signal color and time-remaining queries

```cpp
    unsigned int signal_state_for(int group, double sim_time) const;
    double signal_remaining_for(int group, double sim_time) const;
```

```cpp
Intersection::Intersection(std::string name, double x, double y)
    : name(std::move(name)), x(x), y(y),
      is_building(false), is_roundabout(false),
      radius(0.0), light_system(nullptr) {
    display_name = this->name;  // default the label to the id
}

unsigned int Intersection::signal_state_for(int group, double sim_time) const {
    if (!light_system) return TrafficLight::Green;
    return light_system->state_at(group, sim_time);
}

double Intersection::signal_remaining_for(int group, double sim_time) const {
    if (!light_system) return 0.0;
    return light_system->remaining_at(group, sim_time);
}
```

**Meaning:** The constructor's member-initializer list moves the `name` string in (avoiding a copy),
copies `x`/`y` by value (cheap `double`s), and defaults `is_building`/`is_roundabout` to `false`,
`radius` to `0.0`, `light_system` to `nullptr` explicitly (redundant with `unique_ptr`'s own default
constructor, but stated for clarity). The body sets `display_name` to a copy of `this->name` — using
`this->name` rather than the constructor parameter `name` because the parameter was already moved
from on the initializer list above it, so referring to the now-moved-from parameter would be
incorrect; `this->name` is the class member, already holding the moved-in value.
`signal_state_for`/`signal_remaining_for` both follow the same shape: if there's no `light_system`,
return the "everyone goes" answer (`TrafficLight::Green`, or `0.0` seconds remaining since there's
nothing to count down) — otherwise delegate to the `TrafficLight`'s own `state_at`/`remaining_at`.

**Why:** the comment directly above `signal_state_for` states the design intent: *"A junction with
no light is green to everyone: callers must not have to special-case unsignalized nodes to let
traffic through."* This is the same "eliminate a bug class by construction" philosophy as
`is_signalized()` — rather than every caller in the tick loop writing
`if (node->is_signalized()) { check the light } else { just go }`, the unsignalized case is folded
into the same function so callers always call `signal_state_for()` unconditionally and get the right
answer either way.

**Usage:** This is almost certainly what `TrafficSimulator`'s per-tick logic (and vehicle
decision-making — "can I proceed past this junction?") calls for every vehicle approaching a
junction; see `learn/07-simulation-core.md`. The actual meaning of `TrafficLight::state_at`/
`remaining_at` — how phases, cycle length, and `group` interact — belongs to
[`learn/03-traffic-lights.md`](03-traffic-lights.md).

**Check yourself:**
- Why does the constructor set `display_name = this->name` in the body instead of in the member-initializer list?
- Trace what a vehicle-decision function calling `signal_state_for(group, sim_time)` sees for an unsignalized junction versus a signalized one — does the caller need an `if (is_signalized())` check anywhere?

---

## `cpp_backend/include/core/Road.hpp` / `cpp_backend/src/core/Road.cpp`

### Lines 1–20: `CostMode` and forward declarations

```cpp
class Intersection;
class Vehicle;
class Geometry;

enum class CostMode { Distance, Time };
```

**Meaning:** Three more forward declarations — `Road` only needs pointers to `Intersection`,
`Vehicle`, and `Geometry`, not their full definitions. `CostMode` is a scoped (`enum class`, not a
plain `enum`) enumeration with two values: `Distance` and `Time`. Being scoped means the values must
be written as `CostMode::Distance`, not bare `Distance` — this avoids polluting the surrounding
namespace and prevents accidental implicit conversion to `int`.

**Why:** the comment above the enum explains why it lives here, in `core/Road.hpp`, rather than
next to the `routing/PathFinding` classes that actually consume it: *"it is a property of how you
price a ROAD, and `Road::cost()` is what actually answers it. Routing consumes it, so the dependency
points core -> routing and not back."* This is a deliberate layering decision — `core` doesn't
depend on `routing`, but `routing` depends on `core`, so any type shared between them has to live on
the `core` side of that boundary or the dependency would point the wrong way. The two values
document a real behavioral difference: `Distance` mode gives a stable answer (shortest metres,
ignoring live traffic) while `Time` mode is congestion-sensitive and can genuinely disagree between
two route requests issued a second apart.

**Usage:** Consumed by `Road::cost()` (below) and by whichever routing strategy — `BFS`, `Dijkstra`,
`AStar` (see `learn/04-routing.md`) — needs to decide what "shortest" means for a given
request.

**Check yourself:**
- Why is `CostMode` declared in `core/Road.hpp` instead of `routing/PathFinding.hpp`, given that routing is what actually chooses between `Distance` and `Time`?
- What's the practical difference in results between routing two vehicles a second apart under `CostMode::Time` versus `CostMode::Distance`?

---

### Lines 22–45: topology fields

```cpp
class Road {
public:
    std::string id;
    uint32_t index = 0;
    uint32_t base_index = 0;
    bool reverse_twin = false;

    Intersection* source;
    Intersection* target;

    int capacity;
    bool is_one_way;
    bool is_internal;
    double base_weight;
    double travel_cost;
    double length;

    Geometry* geometry;

    std::vector<Vehicle*> vehicles;
```

**Meaning:** `id` is the string key `TrafficGraph::edges` uses. `index`/`base_index`/`reverse_twin`
are the dense-id fields `TrafficGraph::reindex()` populates (covered above). `source`/`target` are
raw, non-owning pointers to the two `Intersection`s this road connects — non-owning because
`TrafficGraph::intersections` owns them. `capacity` is a vehicle-count ceiling; `is_one_way` records
whether the reverse direction exists as a twin `Road`; `is_internal` flags a road belonging to a
roundabout's internal ring. `base_weight` is the "ideal conditions" travel cost, `travel_cost` the
current (possibly congestion-adjusted) cost, `length` the road's physical length. `geometry` is a
raw, non-owning `Geometry*` (defined elsewhere — likely the curve/shape used for rendering and for
placing vehicles along a non-straight road; not covered in this module). `vehicles` is the live list
of `Vehicle*` currently on this road — again non-owning; vehicles are owned elsewhere (see
`learn/05-vehicles.md`).

**Why:** the block comment right above the private section (further down) explains the philosophy
covering all of these: this is "topology set once at construction and read constantly on the
simulation hot path" — wrapping `length` or `capacity` in a getter would add a function-call
indirection that protects no invariant, since nothing about them needs validating after
construction. Contrast this with the *private* fields below (`lanes_`, `speed_limit_`,
`congestion_level_`, `is_closed_`, `capacity_scale_`, `event_congestion_`) — every one of those
*does* get mutated after construction (by `TrafficEvent`s or live map editing) and every one of
those has a real invariant (a valid numeric range, or "a closed road must report infinite cost") —
so those, and only those, get private storage plus a getter/setter pair.

**Usage:** `source`/`target` are what `Intersection::outgoing_edges`/`incoming_edges` get populated
from at construction (see the constructor below) — this is the actual adjacency structure routing
algorithms walk. `vehicles` is what `TrafficSimulator`'s tick loop and `get_lead_vehicle()` (below)
iterate over.

**Check yourself:**
- Why are `source` and `target` raw pointers rather than `unique_ptr<Intersection>` — what would break if `Road` tried to own its endpoints?
- Contrast `length` (public field) with `lanes_` (private, behind `get_lanes()`/`set_lanes()`) — what's the actual difference in how each is used after construction that justifies the different treatment?

---

### Lines 47–68: signal-approach fields

```cpp
    int signal_group = -1;
    double signal_stop_from_end = 8.0;  // m before the road's end

    bool has_signal_head = false;
    double signal_head_x = 0.0, signal_head_y = 0.0;
    double signal_head_ux = 0.0, signal_head_uy = 0.0;
```

**Meaning:** `signal_group` is `-1` by default ("no signal on this approach"), overwritten by
`TrafficGraph::assign_signal_groups()`. `signal_stop_from_end` defaults to 8 meters, also overwritten
by that same function using the `min(8.0, length * 0.4)` rule seen earlier. `has_signal_head`,
`signal_head_x/y`, and `signal_head_ux/uy` are a separate concern entirely — they only get set when
the *source map data* explicitly places an `highway=traffic_signals` node on this specific approach,
recording where the physical signal head actually stands (`x`/`y`) and the unit direction vector of
the road segment it stands on (`ux`/`uy`), used to draw a stop line at the correct angle across the
carriageway even on a curved road.

**Why:** the comment is explicit about *why* these are two separate concerns rather than one: when a
junction is signalized, *every* incoming approach gets a phase group and a stop line so traffic
actually stops correctly — but drawing a rendered light head on every approach would invent lights
the real city intersection doesn't have, if the source map only tagged one arm. The stopping *logic*
always uses `signal_stop_from_end` regardless of `has_signal_head`; `has_signal_head` only gates
whether the *renderer* draws a head. The unit-vector fields exist because deriving a stop-line
angle from the approach's two endpoints (source and target `Intersection` coordinates) would give
the straight-line chord — fine for a straight road, visibly wrong (crossing the carriageway at an
angle) for a curved or split approach where the actual road geometry bends before reaching the
junction.

**Usage:** `signal_group`/`signal_stop_from_end` feed `stop_line_position()` (below), used by vehicle
decision logic in the tick loop. `has_signal_head`/`signal_head_*` are consumed by the renderer only
— they don't affect simulation behavior.

**Check yourself:**
- Why can a junction have every approach carrying a valid `signal_group` while only some of those approaches have `has_signal_head == true`?
- Why is `signal_head_ux/uy` derived from the local road segment's direction rather than the straight line between the approach's two endpoint intersections?

---

### Lines 70–89: `kClosedRoadCost`, constructor declaration, basic behaviors

```cpp
    static constexpr double kClosedRoadCost = 1e8;

    Road(std::string id, Intersection* source, Intersection* target, int lanes, double speed_limit,
        int capacity = 12, bool is_one_way = false, double length = 0.0,
        Geometry* geometry = nullptr);

    void add_vehicle(Vehicle* vehicle);
    void remove_vehicle(Vehicle* vehicle);
    double get_weight() const;
    void update();
    std::pair<Vehicle*, double> get_lead_vehicle(Vehicle* current_vehicle);
    void update_travel_cost();
```

```cpp
Road::Road(std::string id, Intersection* source, Intersection* target, int lanes, double speed_limit,
           int capacity, bool is_one_way, double length, Geometry* geometry)
    : id(std::move(id)), source(source), target(target),
      capacity(capacity), is_one_way(is_one_way),
      is_internal(false), geometry(geometry),
      lanes_(lanes), speed_limit_(speed_limit) {

    if (length > 0.0) {
        this->length = length;
    } else {
        double dx = target->x - source->x;
        double dy = target->y - source->y;
        this->length = std::sqrt(dx * dx + dy * dy);
    }

    base_weight = this->length;
    travel_cost = base_weight;

    source->outgoing_edges.push_back(this);
    target->incoming_edges.push_back(this);
}
```

**Meaning:** `kClosedRoadCost` is a `static constexpr` — a compile-time constant shared by every
`Road` instance (no per-object storage), set to `1e8`, used as a large-but-finite stand-in cost for a
closed road in `update_travel_cost()` (distinct from the *true* infinite cost `cost()` itself
reports — see below, that distinction matters). The constructor takes `length = 0.0` as a default,
documented by the field comment "A length <= 0 means 'measure it from the endpoints.'" The body
checks exactly that: if the caller passed a real `length > 0.0`, use it (this is the case when the
source map data already carries an actual road length, e.g. following a curved OSM way, which is
longer than the straight-line distance between endpoints); otherwise compute the Euclidean distance
between `source` and `target`'s coordinates. `base_weight` and `travel_cost` both start equal to
`length` — the ideal-conditions cost before any congestion is factored in. The final two lines are
the adjacency-list wiring: this `Road` pushes its own `this` pointer into `source->outgoing_edges`
and `target->incoming_edges`.

**Why:** falling back to Euclidean distance when no explicit length is given is a pragmatic default
for synthetic/test roads that don't come from real map data, while still respecting real curved-road
lengths when they're known — a straight-line measurement between endpoints would understate a
curved road's actual travel distance. The adjacency-list push-back at the end of the constructor is
what makes `Intersection::outgoing_edges`/`incoming_edges` (declared as empty `vector<Road*>` in
`Intersection`'s own constructor) actually populated — no other code path adds to those vectors, so
every `Road` must go through this constructor exactly once for the graph's adjacency to be correct.

**Usage:** `add_vehicle`/`remove_vehicle` (trivial push_back/erase-remove pairs, not shown in detail
since they're single-line bodies) are called by whatever moves a `Vehicle` onto or off of a road —
almost certainly `TrafficSimulator`'s tick loop (see `learn/07-simulation-core.md`) and vehicle
lane-change logic (`learn/05-vehicles.md`). `get_weight()` just returns the current `travel_cost` —
the routing-facing read of "how much does this edge cost right now." `update()` is a one-line
forwarder to `update_travel_cost()` — presumably a uniform "tick this road" hook the simulator calls
on every `Road` each frame, per the mention of "TrafficEvent" behaviors in the comment above it.

**Check yourself:**
- Why does the constructor push `this` into `source->outgoing_edges` and `target->incoming_edges` at the very end of the body, rather than earlier?
- What real-world map situation would make `length` (from the map file) meaningfully larger than the straight-line distance the constructor would otherwise compute?
- Why is `kClosedRoadCost` a large finite number (`1e8`) here, while `cost()`'s closed-road branch (below) returns actual `infinity`? What's the difference in what each function is used for?

---

### Lines 91–176: derived-state functions and the encapsulated mutable fields

```cpp
double Road::get_weight() const {
    return travel_cost;
}

void Road::update() {
    update_travel_cost();
}
```

**Meaning/Why/Usage:** Two thin forwarders. `get_weight()` is a plain read of the cached
`travel_cost` field — routing code that just wants "the current cost of this edge" without caring
whether it's fresh calls this. `update()` exists as a uniform per-tick hook name — likely so
`TrafficSimulator` can call `road->update()` on every `Road` in a loop without needing to know that
what actually happens is `update_travel_cost()`; this indirection would also be the natural place to
add other per-tick road behavior later without changing the call site.

```cpp
std::pair<Vehicle*, double> Road::get_lead_vehicle(Vehicle* current_vehicle) {
    double ref_position = current_vehicle ? current_vehicle->position_on_road : 0.0;

    Vehicle* lead = nullptr;
    double min_gap = std::numeric_limits<double>::infinity();

    for (Vehicle* other : vehicles) {
        if (!other || other == current_vehicle) continue;

        double gap = other->position_on_road - ref_position;
        if (gap > 0.0 && gap < min_gap) {
            min_gap = gap;
            lead = other;
        }
    }

    return {lead, min_gap};
}
```

**Meaning:** Finds the closest vehicle *ahead* of `current_vehicle` on this road by scanning every
vehicle in the `vehicles` list — a linear scan, not a sorted structure. `ref_position` is
`current_vehicle`'s own position, or `0.0` (the start of the road) if `current_vehicle` is `nullptr`
— the documented meaning of that case is "a vehicle about to enter this road," measuring gap from
the road's start rather than from an existing vehicle's position. For each `other` vehicle (skipping
null entries and skipping `current_vehicle` itself so a vehicle never treats itself as its own lead),
`gap` is `other`'s position minus the reference position; only a *positive* gap (strictly ahead)
smaller than the current minimum updates `lead`/`min_gap`. If nothing qualifies, `lead` stays
`nullptr` and `min_gap` stays `+infinity`, both returned together as a `std::pair`.

**Why:** the comment documents the return semantics precisely so callers don't have to guess: "clear
road ahead" is representable without a special sentinel value by returning `(nullptr, +infinity)` —
a caller doing distance comparisons (e.g. "is the gap ahead big enough to change lanes into") can use
`min_gap` directly without first null-checking `lead`. Linear scan over `vehicles` is O(n) in the
number of vehicles on this one road, which is acceptable because a single road realistically holds a
small number of vehicles (bounded by `capacity`), not the whole simulation's vehicle count.

**Usage:** This is almost certainly what car-following logic (IDM — Intelligent Driver Model,
referenced by name in `effective_speed_ms`'s comment below) calls every tick to find out how far
ahead the next vehicle is, feeding that gap into acceleration/braking decisions — see
`learn/05-vehicles.md`.

```cpp
double Road::effective_speed_ms(double sensitivity) const {
    const double slowdown =
        1.0 - 0.50 * congestion_level_ * std::clamp(sensitivity, 0.0, 1.0);
    return std::max(1.0, speed_limit_ms() * slowdown);  // never below 1 m/s
}

double Road::travel_time_seconds(double sensitivity) const {
    return length / effective_speed_ms(sensitivity);
}
```

**Meaning:** `effective_speed_ms` reduces the road's speed limit (converted to m/s via
`speed_limit_ms()`) by a `slowdown` factor driven by `congestion_level_` (a private field, `[0,1]`)
and `sensitivity` (clamped to `[0,1]`, how strongly *this particular caller* is affected by
congestion). At worst — full congestion, full sensitivity — `slowdown` bottoms out at `0.5`, i.e.
50% of the speed limit; the comment header block above this function (in the `.hpp`) says "down to
15% of the limit at worst" but the actual formula (`1.0 - 0.50 * congestion * sensitivity`) floors at
50%, not 15% — worth flagging as a place where the header's prose comment and the `.cpp`'s inline
comment ("Moderate congestion slowdown (down to ~50% of speed limit at worst)") actually agree with
each other but disagree with the header's own doc comment above `effective_speed_ms`'s declaration.
Either way, `std::max(1.0, ...)` guarantees the result is never below 1 m/s regardless of how the
slowdown is tuned. `travel_time_seconds` is just `length / effective_speed_ms(sensitivity)`.

**Why:** the comment explains the floor precisely: a road reporting 0 m/s would make
`travel_time_seconds()` return infinity and would give IDM (car-following model) a target speed of
zero, meaning a jammed road could never un-jam — no vehicle would ever have a nonzero target speed to
accelerate toward. Flooring at 1 m/s keeps the simulation able to recover from its own congestion
instead of modeling a permanent standstill. `sensitivity` exists so a caller like an emergency
vehicle (see `learn/05-vehicles.md`'s `EmergencyVehicle`) can pass a lower value and be less slowed
by the same queue everyone else is stuck in — "an ambulance is barely slowed by a queue," per the
comment.

**Usage:** `travel_time_seconds()` feeds directly into `cost()` under `CostMode::Time` (below) — this
is the live, congestion-aware routing cost. `effective_speed_ms` itself is presumably also what
vehicle physics targets when accelerating on this road (see `learn/05-vehicles.md`).

**Note on the discrepancy above:** flag this honestly rather than smoothing it over — the header
doc comment for this section says congestion can drag speed "to 15% of the limit at worst," but the
formula and the `.cpp`'s own comment both describe a 50% floor. Either the doc comment is stale (was
accurate before a tuning change) or the constant was later adjusted without updating the prose above
it. Either way, trust the code (`1.0 - 0.50 * ...`), not the header's numeric claim, if you're
reasoning about actual behavior.

```cpp
double Road::stop_line_position() const {
    if (signal_group < 0) return length;
    return std::max(0.0, length - signal_stop_from_end);
}
```

**Meaning:** If this approach has no signal (`signal_group < 0`), the "stop line" is just the end of
the road (`length`) — nothing to stop for, so a vehicle can use the road's full length. Otherwise,
it's `length - signal_stop_from_end`, clamped to never go negative.

**Why:** returning `length` (not some sentinel like `-1` or `NaN`) for the unsignalized case is the
same "callers shouldn't need to special-case" philosophy seen in `Intersection::signal_state_for()`
— a caller can call `stop_line_position()` unconditionally and get a physically sensible answer
either way. The `max(0.0, ...)` clamp guards the same short-block problem `assign_signal_groups`
already accounted for when it capped `signal_stop_from_end` at `40%` of `length` — this is
defense in depth: even if `signal_stop_from_end` somehow ended up larger than `length` some other
way (a test setting it directly, for instance), the stop line still can't go negative.

**Usage:** Vehicle decision logic — "am I past the stop line, should I be braking for this signal" —
reads this every tick for any vehicle approaching a signalized junction; see
`learn/07-simulation-core.md`.

```cpp
double Road::cost(CostMode mode) const {
    if (is_closed_) return std::numeric_limits<double>::infinity();
    return mode == CostMode::Distance ? length : travel_time_seconds();
}
```

**Meaning:** If the road is closed, cost is *true* infinity, not a large finite number. Otherwise,
dispatch on `mode`: `Distance` returns raw `length`, `Time` returns the congestion-aware
`travel_time_seconds()`.

**Why:** the header comment explains why this uses real infinity rather than a big finite value like
`kClosedRoadCost`: *"a big number is still cheaper than a long detour, which is how a 'closed' road
ends up on a route anyway."* This is a genuinely important routing correctness point — if a
pathfinder is comparing summed path costs, any finite number, no matter how large, can in principle
be beaten by a sufficiently long legitimate detour, meaning a closed road could still get chosen if
the alternative route is long enough. True infinity has no such failure mode: no finite sum of real
road costs can ever be less than infinity, so a closed road is categorically unroutable. This is a
different function from `kClosedRoadCost` (used inside `update_travel_cost()`, below) — worth
noting as a subtlety: the *cached* `travel_cost` field uses the large-finite sentinel, while the
*on-demand* `cost()` function used for actual routing decisions uses true infinity. That split
matters and is worth sitting with rather than assuming it's simply the same idea expressed twice.

**Usage:** This is what routing strategies (`learn/04-routing.md`) call to price an edge for
a specific request; `mode` is presumably threaded through from whatever the caller (a vehicle
requesting a route, or the routing benchmark tool `route_bench` mentioned in
`learn/01-orientation.md`) actually wants optimized.

```cpp
void Road::update_travel_cost() {
    if (is_closed_) {
        travel_cost = kClosedRoadCost;
        return;
    }

    const double actual_capacity = std::max(1.0, lanes_ * capacity * capacity_scale_);
    const double density = static_cast<double>(vehicles.size()) / actual_capacity;

    const double base_congestion = 0.70 * std::pow(std::min(1.0, density), 1.3);
    congestion_level_ = std::clamp(base_congestion + event_congestion_, 0.0, 1.0);

    travel_cost = base_weight * (1.0 + congestion_level_);
}
```

**Meaning:** If closed, cache `travel_cost` as the large finite sentinel and return early — this is
the function that actually populates the *cached* field `get_weight()` reads, as distinct from
`cost()`'s on-demand true-infinity answer. Otherwise: `actual_capacity` is the road's real vehicle
capacity, scaled by `lanes_`, the base `capacity`, and `capacity_scale_` (an event-driven multiplier,
e.g. an accident narrowing the road — see the setter comments in the header), floored at `1.0` so a
degenerate zero-capacity road can't divide by zero below. `density` is current vehicle count over
that capacity — can exceed `1.0` if the road is over capacity. `base_congestion` maps density through
a power curve (`density^1.3`, density first clamped to `[0, 1]` so overcrowding doesn't blow past the
curve's intended range) scaled to top out at `0.70` — i.e. vehicle density alone can push congestion
up to 70%, never all the way to 100%. `congestion_level_` adds `event_congestion_` (a separate,
event-driven additive term — see `add_event_congestion()`) on top and clamps the *sum* to `[0, 1]` —
so events are what can push a road the rest of the way to full congestion, not density alone.
`travel_cost` is finally recomputed as `base_weight * (1 + congestion_level_)` — at zero congestion
this equals `base_weight` (the ideal-conditions cost); at full congestion it's double.

**Why:** capping density-driven congestion at 70% while letting `event_congestion_` push the rest of
the way to 100% is a real modeling choice: ordinary traffic buildup should never fully choke a road
in this model, but an explicit event (an accident, a closure-adjacent effect) can. The `1.3` exponent
is a tuning constant that makes congestion ramp up faster as density approaches capacity than it does
at low density — a smoother, more realistic-feeling response than a linear relationship, though (like
any such constant) it's an unexplained magic number in the sense that nothing in the comments derives
`1.3` from first principles; it reads as tuned-by-feel.

**Usage:** Called from `update()`, which per the earlier discussion is likely invoked once per tick
per road by `TrafficSimulator` — this is the function that actually keeps `travel_cost` (read by
`get_weight()`) and `congestion_level_` (read by `effective_speed_ms()` and thus
`travel_time_seconds()` and thus `cost(CostMode::Time)`) fresh as vehicles move on and off the road
over time.

---

### Lines 90–176: the encapsulated mutable fields (getters/setters)

```cpp
int get_lanes() const { return lanes_; }
void set_lanes(int n) { lanes_ = n < 1 ? 1 : n; }

double get_speed_limit() const { return speed_limit_; }
void set_speed_limit(double kmh) { speed_limit_ = kmh < 5.0 ? 5.0 : kmh; }

bool is_closed() const { return is_closed_; }
void set_closed(bool closed) { is_closed_ = closed; }

double get_congestion_level() const { return congestion_level_; }
void set_congestion_level(double c) {
    congestion_level_ = c < 0.0 ? 0.0 : (c > 1.0 ? 1.0 : c);
}

double get_capacity_scale() const { return capacity_scale_; }
void scale_capacity(double factor) {
    capacity_scale_ *= factor;
    if (capacity_scale_ > 1.0) capacity_scale_ = 1.0;
    if (capacity_scale_ < 0.01) capacity_scale_ = 0.01;
}

double get_event_congestion() const { return event_congestion_; }
void add_event_congestion(double delta) {
    event_congestion_ += delta;
    if (event_congestion_ < 0.0) event_congestion_ = 0.0;
}
```

**Meaning:** Six pairs of getter + setter, every one enforcing a floor (and sometimes a ceiling) on
its private field: lane count never below `1`; speed limit never below `5.0` km/h; congestion level
clamped to `[0, 1]`; `capacity_scale_` clamped to `[0.01, 1.0]` (never zero — "never fully seized," a
direct quote from the comment, meaning even a maximally-damaged road retains 1% capacity rather than
becoming literally impassable through this mechanism — closure is a separate, explicit `is_closed_`
flag for that); `event_congestion_` floored at `0.0` (never negative). `is_closed()`/`set_closed()`
is the plain boolean pair with no clamping needed.

**Why:** the block comment above the private section (quoted partially already) draws the line
precisely: these are "the fields that actually change after construction — `TrafficEvent`
(accidents, closures, congestion) and live map editing (`update_road()`) mutate them, each under an
invariant worth protecting." Two of the setters are worth calling out for *how* they combine, per
their own comments: `scale_capacity(factor)` **multiplies** the existing scale rather than setting it
outright, specifically so "two overlapping accidents compose instead of the second clobbering the
first's effect" — if one accident cuts capacity by 50% and a second, independent accident also cuts
it by 50%, the road ends up at 25% capacity, and clearing one accident (multiplying back by its
inverse) restores exactly the other's effect, not a value that erased both. `add_event_congestion`
is **additive** for the same compositional reason on the other metric — two overlapping congestion
events must stack and unwind independently. This multiplicative-vs-additive distinction is not
incidental; it's the only way either mechanism supports more than one active event at a time without
one event's `set_...()` call silently erasing another's.

**Usage:** `TrafficEvent` handlers (see `learn/06-events-observer.md`) are the obvious caller —
an accident event presumably calls `scale_capacity(0.5)` or similar and later un-does it by scaling
by the reciprocal; a congestion event calls `add_event_congestion(delta)` and later subtracts the
same `delta`. `set_congestion_level()` is explicitly documented as existing for tests and scripted
scenarios wanting to force a value directly, separate from the normal path where
`update_travel_cost()` computes it from live density.

**Check yourself:**
- Why is `scale_capacity()` multiplicative while `add_event_congestion()` is additive — what would go wrong if both were "set" operations instead?
- What's the difference between a road with `capacity_scale_ == 0.01` (from `scale_capacity`) and a road with `is_closed_ == true` — are they interchangeable for routing purposes? (Check `cost()` and `update_travel_cost()` again.)
- The header doc comment above `effective_speed_ms()` claims congestion can reduce speed "to 15% of the limit at worst," but the formula floors at 50%. Which should you trust when reasoning about actual runtime behavior, and why?

---

## `cpp_backend/include/core/Building.hpp` / `cpp_backend/src/core/Building.cpp`

### Lines 6–18: construction and read-only accessors

```cpp
class Building {
public:
    Building(std::string name, std::string type, int levels,
             std::vector<std::pair<double, double>> footprint);

    const std::string& get_name() const { return name_; }
    const std::string& get_type() const { return type_; }
    int get_levels() const { return levels_; }

    const std::vector<std::pair<double, double>>& get_footprint() const { return footprint_; }
```

**Meaning:** Unlike `Intersection` and `Road`'s topology fields, every one of `Building`'s data
members is private, exposed only through `const`-returning getters — `get_name()`/`get_type()`
return `const std::string&` (a reference, avoiding a string copy on every read), `get_levels()`
returns `int` by value (cheap enough that a reference would gain nothing), `get_footprint()` returns
`const std::vector<std::pair<double,double>>&` — a reference to the whole polygon, avoiding copying
potentially many points on every read.

**Why:** this is a real style contrast worth noticing against `Intersection`/`Road`: those two make
topology fields public because nothing about `x`, `y`, or `length` needs protecting after
construction. `Building`, by contrast, makes *everything* private-with-getters, even fields with no
obvious invariant (a `type` string, a `levels` count) — a stricter, more conventional
encapsulation style than the rest of this module uses. The codebase doesn't explain this
inconsistency directly; a reasonable read is that `Building` was written with an eye toward
"nothing except `MapLoader` should ever construct or mutate one" (reinforced by the constructor
being the *only* general-purpose way to set most fields — see `node_id_` below, which gets an
explicit setter precisely because it's the one field that legitimately needs to change after
construction), whereas `Intersection`/`Road` are mutated piecemeal by many different subsystems on
the simulation hot path where an accessor's cost (however small) is less acceptable.

**Usage:** The renderer reads `get_type()` to decide how to draw a building (the comment lists
example values: `"commercial"`, `"hospital"`, `"residential"`), `get_levels()` presumably to draw a
building taller or with more visual detail, `get_footprint()` to draw its actual outline polygon.

**Check yourself:**
- Why does `Building` return `const std::string&` from `get_name()`/`get_type()` but `get_levels()` returns `int` by value? What's the underlying rule for when a reference return is worth it?
- Contrast `Building`'s all-private-with-getters style with `Intersection`'s all-public-fields style — what's a plausible reason for the difference, given how each is used?

---

### Lines 20–36: the anchor node id and private storage

```cpp
    const std::string& get_node_id() const { return node_id_; }
    void set_node_id(std::string node_id) { node_id_ = std::move(node_id); }

    std::pair<double, double> centroid() const;

private:
    std::string name_;
    std::string node_id_;
    std::string type_;
    int levels_;
    std::vector<std::pair<double, double>> footprint_;
};
```

**Meaning:** `node_id_` — which `Intersection` (by id string) this building is anchored to, or
empty if it's display-only with no graph relationship — is the one field with both a getter *and* a
setter. `set_node_id` takes its parameter by value and moves it in, the standard idiom for a setter
that always ends up owning the string regardless of whether the caller passed an lvalue or rvalue.
`centroid()` is declared here, defined in the `.cpp`. The private section lists all five backing
fields plainly, in the same order the public getters expose them (plus `node_id_`, which sits between
`name_` and `type_` here but is accessed through a getter positioned after `get_levels()`/
`get_footprint()` in the public section — the declaration order of private fields doesn't need to
match the public accessor order, and doesn't here).

**Why:** the comment explains precisely why `node_id_` breaks the "everything set once at
construction" pattern the rest of the class follows: it's "not a constructor parameter: `MapLoader`
sets it once, right after construction, only when the map file specifies one." A building with no
anchor in the source data simply never gets `set_node_id()` called, leaving `node_id_` as its
default-constructed empty string. The comment also explains the payoff of a setter over a public
field here specifically: "a setter (rather than a public field) still means nothing else in the
codebase can reassign a building's anchor later" — encapsulation isn't just about
construction-time convenience, it's a statement that *only* the loading path is expected to ever
touch this value, even though technically a setter can be called from anywhere; it's a documented
contract more than a hard guarantee.

```cpp
std::pair<double, double> Building::centroid() const {
    if (footprint_.empty()) return {0.0, 0.0};

    double sum_x = 0.0, sum_y = 0.0;
    for (const auto& point : footprint_) {
        sum_x += point.first;
        sum_y += point.second;
    }
    const double n = static_cast<double>(footprint_.size());
    return {sum_x / n, sum_y / n};
}
```

**Meaning:** If there's no footprint polygon at all, return `(0, 0)` rather than dividing by zero.
Otherwise, sum every point's `x` (`.first`) and `y` (`.second`) and divide by the count — the
arithmetic mean of the polygon's vertices. Note this is the mean of the *vertices*, not the true
area-weighted centroid of the polygon (which would need a different, area-based formula) — for a
roughly regular building footprint the difference is negligible, but for an irregular polygon with
unevenly spaced vertices this can be measurably off from the geometric center.

**Why:** the `(0.0, 0.0)` fallback for an empty footprint (documented directly in the header comment
— "returns (0, 0) when there is no footprint") is a defensive default for a building that's only
anchored to a node (no polygon at all) — the header comment for `get_footprint()` confirms that case
exists ("Empty when the building is only anchored to a node"). Using the simple vertex-average rather
than a true polygon centroid is presumably "good enough" for the renderer's purposes (placing a label
or icon near the middle of the building) without the added complexity of a proper area-weighted
calculation.

**Usage:** Likely used by the renderer to place a building's label or icon, and possibly by
`MapLoader`/`MapEditor` when a building has no explicit anchor node and something needs an
approximate position for it anyway.

**Check yourself:**
- What does `centroid()` return for a building with an empty `footprint_`, and why is that the right fallback given `get_footprint()`'s documented meaning?
- Is `centroid()` the true geometric center of the polygon, or something cheaper? For what kind of footprint shape would the difference actually matter?
- Why does `set_node_id` take its parameter by value and `std::move` it in, rather than taking `const std::string&`?

---

## `cpp_backend/include/core/Zone.hpp`

### Lines 1–13: the entire file

```cpp
#pragma once
#include <string>

// Display-only background region ("CITY CENTER", "PARK", ...).
// The map stores a zone as a polygon; we keep only its bounding box.
struct Zone {
    std::string name;
    std::string kind;
    double x = 0.0;  // top-left corner
    double y = 0.0;
    double w = 0.0;
    double h = 0.0;
};
```

**Meaning:** `Zone` is declared with `struct`, not `class` — in C++ the only difference is default
member access (`public` for `struct`, `private` for `class`), and here that's exactly the point:
every field is meant to be public data with no invariant to protect, so `struct` communicates that
honestly rather than writing `class Zone { public: ... };` for the same effect. Six fields: `name`
and `kind` (both strings — `kind` is presumably the same role as `Intersection::zone`'s string values,
like `"downtown"` or `"park"`), and four `double`s with in-class default member initializers (`= 0.0`)
— `x`/`y` for the top-left corner, `w`/`h` for width/height, together fully describing an
axis-aligned bounding box.

**Why:** the comment states the key simplification directly: "The map stores a zone as a polygon; we
keep only its bounding box." Whatever polygon the source GeoJSON actually describes for a zone (a
park's irregular outline, say) is reduced, at load time, to the smallest enclosing rectangle. This is
consistent with `Zone`'s documented role as "display-only" — it's background decoration for the
renderer, not something routing or vehicle logic ever queries geometrically, so a bounding box is
cheap and sufficient: no per-vehicle "am I inside this zone's actual polygon" test is needed anywhere
in the simulation, only "roughly where does this label/tint go."

**Usage:** `TrafficGraph::zones` is a `std::vector<Zone>`, populated by `MapLoader` while reading a
map's zone polygons and reducing each to its bounding box (see `learn/08-supporting-systems.md` for
the reduction itself). The renderer iterates that vector to draw background regions. Because every
field defaults to `0.0` via in-class initializers and the class has no user-declared constructor,
`Zone` is an aggregate — it can be built with brace-initialization (`Zone{"Downtown", "downtown", 10,
10, 200, 150}`), which is presumably how `MapLoader` constructs each one.

**Why there's no `Zone.cpp`:** confirmed directly by listing `cpp_backend/src/core/` (only
`Building.cpp`, `Graph.cpp`, `Intersection.cpp`, `Road.cpp`, `TrafficLight.cpp` exist there) and by
`CMakeLists.txt`'s `BACKEND_SOURCES` list (see `learn/01-orientation.md`), which never lists a
`Zone.cpp`. This isn't an oversight — `Zone` has no member functions at all, only data members with
in-class default initializers and a compiler-generated aggregate constructor/destructor. There is
nothing to define out of line, so no `.cpp` is needed; had `Zone` ever grown a member function with a
nontrivial body, that function would need a `.cpp` to live in (and that new `.cpp` would need adding
to `BACKEND_SOURCES`, per the "empty stubs are omitted" policy `learn/01-orientation.md` documents).

**Check yourself:**
- Why is `Zone` declared as a `struct` rather than a `class`, and does that choice change anything functionally?
- Why does `TrafficGraph` only keep a `Zone`'s bounding box instead of its full polygon, given that `Intersection`/`Road`/`Building` all keep richer geometric data?
- If `Zone` later needed a member function like `bool contains(double px, double py) const`, would that force the project to add a `Zone.cpp`? Why or why not, referring to the CMake policy from `learn/01-orientation.md`.

---

## Module summary

The graph is deliberately simple: `TrafficGraph` owns everything (`Intersection`, `Road`,
`Building` via `unique_ptr`; `Zone` by value), and every other subsystem — routing, vehicles, the
tick loop, the renderer — holds non-owning raw pointers or references into it. Two style patterns
recur and are worth carrying forward into later modules: **derive, don't duplicate** (`is_signalized()`,
`stop_line_position()`, `cost()`'s infinity vs. `update_travel_cost()`'s finite sentinel — each
computes its answer from one source of truth instead of caching a second fact that could drift out
of sync), and **public data for hot-path topology, private data with invariant-enforcing accessors
for anything that changes after construction** (`Road`'s public `length`/`capacity` vs. private
`lanes_`/`is_closed_`; `Building`'s stricter all-private style, which is the one real inconsistency
in the module and is best explained by `Building` having a single, disciplined writer — `MapLoader`
— rather than many hot-path mutators).

The one honest rough edge surfaced directly by the code: `TrafficGraph::intersections` has no
spatial index and no collision check at insertion, and `assign_signal_groups()`'s bearing math
assumes source and target intersections sit at meaningfully distinct coordinates — neither guards
against two junctions overlapping, which lines up with the project's documented "junction
overlapping can happen" limitation. A second, smaller inconsistency: the header doc comment above
`Road::effective_speed_ms()` claims a 15%-of-limit worst case, but the actual formula (and the
`.cpp`'s own inline comment) floors congestion slowdown at 50% — trust the code over the header
prose there.

Next: [`learn/03-traffic-lights.md`](03-traffic-lights.md) covers `TrafficLight` itself — the object
`Intersection::light_system` owns and `TrafficGraph::signalize()` constructs — followed by
[`learn/04-routing.md`](04-routing.md), where `Road::cost()`/`CostMode` and the
`source`/`outgoing_edges`/`incoming_edges` adjacency built in this module get walked by `BFS`,
`Dijkstra`, and `AStar`.
</content>
