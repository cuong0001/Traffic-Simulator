# Module 04 — Routing / Strategy Pattern

**Files covered:**
- [`cpp_backend/include/routing/PathFinding.hpp`](../cpp_backend/include/routing/PathFinding.hpp)
- [`cpp_backend/include/routing/BFS.hpp`](../cpp_backend/include/routing/BFS.hpp) / [`cpp_backend/src/routing/BFS.cpp`](../cpp_backend/src/routing/BFS.cpp)
- [`cpp_backend/include/routing/Dijkstra.hpp`](../cpp_backend/include/routing/Dijkstra.hpp) / [`cpp_backend/src/routing/Dijkstra.cpp`](../cpp_backend/src/routing/Dijkstra.cpp)
- [`cpp_backend/include/routing/AStar.hpp`](../cpp_backend/include/routing/AStar.hpp) / [`cpp_backend/src/routing/AStar.cpp`](../cpp_backend/src/routing/AStar.cpp)

This module builds on the graph vocabulary from
[`learn/02-core-graph-model.md`](02-core-graph-model.md): `TrafficGraph`,
`Intersection`, and `Road` are the nouns; this module is the verbs — how a
vehicle actually finds a sequence of intersections to drive through. All four
files together implement one Gang-of-Four pattern, the **Strategy Pattern**:
one interface, three interchangeable algorithms, one caller (`Vehicle`, see
[`learn/05-vehicles.md`](05-vehicles.md)) that never has to know which
concrete algorithm it's holding.

A note before we start: I checked `CMakeLists.txt`'s `BACKEND_SOURCES` list
(see [`learn/01-orientation.md`](01-orientation.md)) for a `PathFinding.cpp`.
There isn't one — only `Dijkstra.cpp`, `BFS.cpp`, and `AStar.cpp` are listed
under `cpp_backend/src/routing/`. That's consistent with what you're about to
read: `PathFinding.hpp` has no function bodies that need a translation unit
of their own — every member is either pure virtual (no body at all) or
defined inline in the header (`set_cost_mode`, `cost_mode`,
`nodes_explored`). There is nothing left for a `.cpp` file to contain, so —
exactly like `core/Zone.hpp` in Module 02 — this is a legitimately
header-only class, not a missing file.

---

### `cpp_backend/include/routing/PathFinding.hpp`

This is the Strategy interface. Every concrete algorithm in this module
inherits from it, and every caller programs against it, never against `BFS`,
`Dijkstra`, or `AStar` by name (with one deliberate exception you'll see in
the Usage section below).

```cpp
#pragma once
#include <cstddef>
#include <string>
#include <vector>

#include "core/Road.hpp"

class TrafficGraph;
class Intersection;
```

**Meaning:** The usual include guard, then three headers/forward-declarations.
`<cstddef>` is pulled in for `std::size_t` (used later for
`nodes_explored_`). `"core/Road.hpp"` is a *real* include, not a forward
declaration — you'll see why in a moment: this header needs `Road.hpp`'s
`CostMode` enum by value, not just by pointer, so a forward declaration
wouldn't be enough. `TrafficGraph` and `Intersection`, by contrast, are only
ever referenced here as pointers or const references, so forward-declaring
them is sufficient and keeps this header light — anyone who only needs the
`PathFinding` interface doesn't have to drag in the full `Graph.hpp` or
`Intersection.hpp` definitions.

**Why:** This is the standard C++ discipline of only including what a header
needs *by value*, and forward-declaring everything it only touches *by
reference or pointer*. It keeps compile times down and avoids circular
includes — `Graph.hpp` itself doesn't need to know about `PathFinding` at
all, so there's no risk of `PathFinding.hpp` and `Graph.hpp` including each
other.

**Usage:** Any file that wants to call `find_path` polymorphically only needs
this header. `Vehicle.hpp`, for instance, includes `"routing/PathFinding.hpp"`
and stores a `std::unique_ptr<PathFinding>` — it never needs to know
`BFS.hpp` or `Dijkstra.hpp` exist.

---

```cpp
// Strategy interface for pathfinding algorithms
class PathFinding {
public:
    virtual ~PathFinding() = default;

    virtual std::vector<Intersection*> find_path(const TrafficGraph& graph,
                                         const std::string& start_id,
                                         const std::string& end_id) = 0;
```

**Meaning:** `PathFinding` is an abstract base class — `find_path` is a pure
virtual function (`= 0`), so `PathFinding` itself can never be instantiated;
only a class that overrides `find_path` (like `BFS`, `Dijkstra`, `AStar`) can
be. `virtual ~PathFinding() = default;` is a virtual destructor with a
compiler-generated body. The signature: given a graph, a start intersection
id, and an end intersection id (both `std::string`, matching
`Intersection::name`'s type from Module 02), return the ordered list of
`Intersection*` that make up the route, from start to end inclusive.

**Why:** The virtual destructor is not optional here — `Vehicle` holds a
`PathFinding` through a `std::unique_ptr<PathFinding>` (see Usage below), and
when that pointer is destroyed or reassigned, it calls `delete` through the
*base* pointer type. Without a virtual destructor, that `delete` would only
run `~PathFinding()` and skip `~BFS()`/`~Dijkstra()`/`~AStar()` — for these
three classes specifically that wouldn't leak anything today (their only
members are STL containers with their own destructors, invoked regardless),
but it is undefined behavior per the standard to delete through a base
pointer with a non-virtual destructor, and it's the kind of bug that stays
silent right up until someone adds a `FILE*` or raw-owning member to a
strategy. Making `find_path` pure virtual (rather than virtual with a default
body) is a deliberate design statement: there is no sensible default
pathfinding algorithm, so every concrete strategy is *required* to supply
one, enforced at compile time.

Returning `std::vector<Intersection*>` (not `std::vector<std::string>` or a
custom `Route` type) means the caller gets live pointers into the graph it
already owns — no copying `Intersection` objects, no re-looking-up ids after
the fact. The cost is that the returned pointers are only valid as long as
the `TrafficGraph` they point into is; nothing in this header stops you from
calling `find_path`, mutating the graph, and then dereferencing a stale
pointer. That's a real sharp edge, but it's consistent with the rest of the
core graph model (see Module 02), which is generally pointer-heavy and
assumes single-threaded, disciplined ownership rather than defensive copying.

**Usage:** Every concrete strategy below overrides exactly this one method.
It's also the *only* method any of the three subclasses is required to
implement — everything else on `PathFinding` (cost mode, node-count
telemetry) is inherited as-is.

---

```cpp
    // What a route should minimise. Default Time: speed limits and live
    // congestion both matter, which is how the simulator prices roads.
    void set_cost_mode(CostMode mode) { cost_mode_ = mode; }
    CostMode cost_mode() const { return cost_mode_; }
```

**Meaning:** Two small inline accessors around a protected member,
`cost_mode_` (declared below). `CostMode` is an enum defined in
`core/Road.hpp`: `enum class CostMode { Distance, Time };`. `set_cost_mode`
is a plain setter; `cost_mode()` is a `const` getter (it doesn't, and
shouldn't, mutate the object).

**Why:** This is *not* virtual — there's no reason for a subclass to override
what is fundamentally just "store a value, read it back." Every concrete
strategy reads `cost_mode_` through `Road::cost(cost_mode_)` (you'll see this
identically in `BFS`, `Dijkstra`, and `AStar`'s edge-expansion loops), so
putting the storage and accessors on the base class means the mode-switching
logic is written once, not duplicated three times. It's a small but real
example of the base class doing more than defining an interface — it's also
sharing state and behavior that all strategies need identically, which is
exactly what a non-pure virtual base is for.

**Usage:** `Vehicle::set_cost_mode` (in `Vehicle.cpp`) forwards straight to
this: `if (router_) router_->set_cost_mode(mode);`. So a vehicle can ask to
minimize either raw `Distance` (road length) or `Time` (which folds in speed
limits *and* live congestion — see `Road::cost()` and
`Road::update_travel_cost()` in Module 02) without knowing or caring which
concrete algorithm `router_` currently points to.

---

```cpp
    // How many nodes the last find_path() call settled. This is the search
    // effort the algorithm-comparison table reports: it is what separates A*
    // from Dijkstra, since both return the same route but A* should reach it
    // after expanding far fewer nodes.
    //
    // Deliberately NOT part of the return type: changing find_path()'s
    // signature would touch every call site across the test suite and buy
    // nothing else.
    std::size_t nodes_explored() const { return nodes_explored_; }

protected:
    CostMode cost_mode_ = CostMode::Time;
    // Each strategy resets this at the top of find_path() and increments it
    // once per node it settles (pops and commits to), never per edge.
    std::size_t nodes_explored_ = 0;
};
```

**Meaning:** `nodes_explored()` is another `const` getter, this time over
`nodes_explored_`, a `protected` (not `private`) `std::size_t` that
subclasses write to directly rather than through a setter. `cost_mode_`
defaults to `CostMode::Time` — the comment above `set_cost_mode` explains
why: unless told otherwise, a route should account for speed limits and
current congestion, not just raw distance.

**Why:** Making `nodes_explored_` `protected` rather than `private` is a
deliberate looseness — it lets `BFS::find_path`, `Dijkstra::find_path`, and
`AStar::find_path` increment it directly (`++nodes_explored_;`) inside their
tight search loops, without going through a setter call on every iteration.
For a counter incremented potentially thousands of times per query, that's a
reasonable place to trade a bit of encapsulation for directness — the
alternative (a `protected void increment_explored()` wrapper) would add a
function call for no real safety benefit, since these are the *only*
classes that could ever legitimately touch this field. The comment nailing
down the counting convention ("once per node it settles ... never per edge")
matters a lot: it's what makes the three algorithms comparable to each other
at all. If BFS counted enqueues and Dijkstra counted dequeues, comparing
`nodes_explored()` across them would be meaningless. You'll see all three
`.cpp` files honor this convention identically: the increment happens right
after popping a node off the frontier and confirming it's not a stale/already
-settled entry, never while pushing.

The design note about *not* folding this into `find_path`'s return type is
worth taking at face value: it's a real API-design tradeoff, and the comment
tells you which side of it they chose and why — a `struct RouteResult { path,
nodes_explored }` return type would be arguably more idiomatic, but it would
touch `find_path`'s signature everywhere it's called (test suite,
`Vehicle::run_router`, any benchmarking tool like `route_bench`) for a
feature that's purely diagnostic. Storing it as post-call state you read
through a separate getter is less "pure," but cheaper to add without
churning the rest of the codebase.

**Usage:** `route_bench` (the benchmarking tool from
[`learn/01-orientation.md`](01-orientation.md)'s CMake target list) is very
likely what calls `nodes_explored()` to build the "algorithm-comparison
table" the top comment references — it's the concrete evidence for the claim
"A* should reach [the goal] after expanding far fewer nodes [than Dijkstra]."

**Check yourself:**
- Why must `find_path` be pure virtual instead of virtual with an empty
  default body?
- What specifically goes wrong if `~PathFinding()` were *not* virtual, given
  that `Vehicle` stores a `std::unique_ptr<PathFinding>`?
- Why is `cost_mode_` stored and manipulated on the base class instead of
  being duplicated in `BFS`, `Dijkstra`, and `AStar` separately?
- The comment says `nodes_explored_` increments "once per node it settles...
  never per edge." Why would counting edge relaxations instead make BFS,
  Dijkstra, and AStar's numbers impossible to compare fairly?

---

### `cpp_backend/include/routing/BFS.hpp`

The first concrete strategy: unweighted breadth-first search.

```cpp
#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include "routing/PathFinding.hpp"

class TrafficGraph;
class Intersection;
class Road;

class BFS final : public PathFinding {
public:
    std::vector<Intersection*> find_path(const TrafficGraph& graph,
                                 const std::string& start_id,
                                 const std::string& end_id) override;

private:
    std::queue<Intersection*> front;
    std::unordered_set<std::string> visited;
    std::unordered_map<std::string, Intersection*> prev;
    std::unordered_map<std::string, Intersection*> node_of;
    std::vector<Intersection*> reconstruct_path(const std::string& start_id,
                                         const std::string& end_id);
};
```

**Meaning:** `class BFS final : public PathFinding` — public inheritance from
the Strategy interface, and `final` tells the compiler (and the reader) that
nothing is ever meant to derive further from `BFS`. `override` on
`find_path` is a compile-time check: it verifies this signature really does
match a virtual function in the base class, catching typos (a wrong
parameter type here would silently create an unrelated overload instead of
overriding, without `override`). The four private members are BFS's working
state: `front` is the FIFO frontier (`std::queue`), `visited` is the "have we
ever enqueued this node" set, `prev` maps an intersection id to the node it
was reached from (for path reconstruction), and `node_of` maps an id back to
its live `Intersection*` (so reconstruction doesn't need to re-look the id up
in the graph). `reconstruct_path` is a private helper, not part of the public
interface — callers only ever see `find_path`.

**Why:** Storing `front`, `visited`, `prev`, and `node_of` as *member*
variables rather than locals inside `find_path` means each `BFS` instance
keeps its scratch containers alive (and their heap allocations reusable)
across calls, instead of reallocating a fresh queue/map/set on every single
`find_path` invocation. Since a `Vehicle` owns its `router_` for its whole
lifetime and reroutes repeatedly (see `Vehicle::request_reroute`), this
avoids repeated allocation churn on a hot path. The cost is that `find_path`
must explicitly clear all four containers at the top of every call (you'll
see this in the `.cpp`) — state that leaks between calls would silently
corrupt future searches, so that reset is not optional.

`std::unordered_set<std::string> visited` alongside `std::unordered_map<...,
Intersection*> node_of` looks redundant — both are effectively keyed on the
same ids — but they serve different questions: `visited` answers "have I
already decided to explore this node," an O(1) existence check that's
consulted for *every* neighbor of *every* expanded node, while `node_of`
answers "give me the actual pointer for this id" only when reconstructing
the final path. Splitting them keeps the hot inner loop's existence check as
cheap as possible (`unordered_set::find` avoids paying for the pointer
payload on every check) — a small optimization but a deliberate one for the
tightest loop in this file.

**Usage:** `final` matters for the Strategy Pattern specifically: it commits
to the shape "BFS is a leaf strategy, not a family of strategies," so nobody
building a hierarchy of BFS variants can quietly subclass it and be handed
around anywhere a `BFS` is expected. `override` is what keeps `BFS` in sync
with `PathFinding` — if `PathFinding::find_path`'s signature ever changes,
every `override` in `BFS`, `Dijkstra`, and `AStar` fails to compile instead
of silently drifting into three unrelated, un-overriding functions.

**Check yourself:**
- Why keep `front`, `visited`, `prev`, and `node_of` as member variables
  instead of local variables inside `find_path`? What would need to change
  in the `.cpp` if they were local?
- What compiler error would you get, and where, if `find_path`'s
  parameter list here didn't exactly match `PathFinding::find_path`'s, given
  the `override` keyword?

---

### `cpp_backend/src/routing/BFS.cpp`

```cpp
std::vector<Intersection*> BFS::find_path(const TrafficGraph& graph,
                                   const std::string& start_id,
                                   const std::string& end_id) {
    // clear state
    while (!front.empty()) front.pop();
    visited.clear();
    prev.clear();
    node_of.clear();
    nodes_explored_ = 0;
```

**Meaning:** `std::queue` has no `clear()` method, so it's drained manually
with a `while` loop calling `pop()` until `empty()`. The three maps/sets have
`clear()`. `nodes_explored_` (inherited, `protected`) is reset to zero — this
is the concrete fulfillment of the promise in `PathFinding.hpp`'s comment,
"Each strategy resets this at the top of find_path()."

**Why:** This is exactly the "explicit reset because these are reused member
containers" cost flagged in the header walkthrough above. Every one of BFS,
Dijkstra, and AStar's `find_path` bodies opens with an identical-in-spirit
block — it's a repeated pattern across all three files precisely because all
three reuse member scratch state the same way.

**Usage:** If this reset were ever forgotten (say, on a future fifth
container added to the class), a vehicle's *second* call to `find_path`
would silently see leftover state from its first call — a bug that would be
very hard to notice, since `visited`/`prev` entries from a previous, related
query might even produce a *plausible-looking* wrong route rather than an
obvious crash.

---

```cpp
    auto start_it = graph.intersections.find(start_id);
    auto end_it   = graph.intersections.find(end_id);
    if (start_it == graph.intersections.end() || end_it == graph.intersections.end())
        return {};

    Intersection* start_node = start_it->second.get();
```

**Meaning:** `graph.intersections` is the `std::unordered_map<std::string,
std::unique_ptr<Intersection>>` from `TrafficGraph` (Module 02). `.find(id)`
does an O(1) average hash lookup; comparing the result against `.end()` is
the standard idiom for "does this key exist." If either id is missing, the
function returns an empty, default-constructed `std::vector<Intersection*>`
immediately — no exception, no partial result. `start_it->second` is the
`std::unique_ptr<Intersection>` stored in the map; `.get()` extracts the raw
`Intersection*` without transferring ownership.

**Why:** Validating both ids *before* doing any search work means a caller
that passes a typo'd or stale id gets a clean, cheap `{}` back instead of the
function crashing when it later tries to dereference a null iterator's
`->second`. Using `.get()` rather than releasing or copying the
`unique_ptr` respects the graph's ownership model from Module 02: the graph
owns its intersections; `BFS` (and every routing algorithm) only ever
borrows non-owning raw pointers into it, which is why the returned
`std::vector<Intersection*>`'s lifetime is tied to the graph outliving it, as
noted in the interface walkthrough above.

**Usage:** This exact two-line existence check (`graph.intersections.find(id)
== graph.intersections.end()`) reappears verbatim in `Dijkstra::find_path`
and `AStar::find_path` — it's the shared entry contract all three strategies
honor, even though nothing in `PathFinding.hpp` forces them to (it's a
convention, not a compiler-enforced one).

---

```cpp
    // Seed the BFS
    front.push(start_node);
    visited.insert(start_id);
    prev[start_id] = nullptr;
    node_of[start_id] = start_node;
```

**Meaning:** The starting node is pushed onto the queue, marked visited, and
recorded as having no parent (`nullptr` — the sentinel meaning "this is where
the walk-back should stop"), with its pointer cached in `node_of`.

**Why:** Marking the start node visited *before* the main loop begins (not
inside it) is what prevents BFS from ever re-enqueueing the start node if
some road happens to loop back to it — without this, a cyclic road graph (a
grid of two-way streets, which is exactly what this simulator models) could
send BFS back through its own starting point.

**Usage:** `prev[start_id] = nullptr` is the base case that
`reconstruct_path` (below) checks for to know when to stop walking backward.

---

```cpp
    while (!front.empty()) {
        Intersection* current = front.front();
        front.pop();

        ++nodes_explored_;   // dequeued: this node is now expanded

        // Goal check
        if (current->name == end_id)
            return reconstruct_path(start_id, end_id);
```

**Meaning:** The classic BFS loop: pop the front of the FIFO queue, count it
as explored, and check whether it's the goal. `front.front()` peeks the
oldest-pushed element; `front.pop()` removes it — two separate calls because
`std::queue::pop()` returns `void` (unlike, say, a stack-based
implementation that might combine peek-and-remove).

**Why:** The goal check happens *after* popping, not at push time — this is
what makes `nodes_explored_` mean "nodes actually expanded" rather than
"nodes discovered," matching the counting convention documented in
`PathFinding.hpp`. Checking for the goal here (rather than only when it's
first discovered as a neighbor) also guarantees BFS returns as soon as the
goal is dequeued, which — because BFS is FIFO — is provably the first time
it's reached via the fewest possible *edges* (not necessarily fewest cost;
see below).

**Usage:** This early-return is the only place `find_path` produces a
non-empty result on success; every other path through the function returns
`{}`.

---

```cpp
        // Expand neighbours via outgoing roads
        for (Road* road : current->outgoing_edges) {
            if (road->is_closed()) continue;   // closed road is untraversable
            Intersection* neighbour = road->target;
            if (visited.find(neighbour->name) == visited.end()) {
                visited.insert(neighbour->name);
                prev[neighbour->name] = current;   // record parent
                node_of[neighbour->name] = neighbour;
                front.push(neighbour);
            }
        }
    }

    // if there's no path found
    return {};
}
```

**Meaning:** For each `Road*` in `current->outgoing_edges` (a
`std::vector<Road*>` on `Intersection`, from Module 02), skip it if
`is_closed()` is true, otherwise look at `road->target` (the `Intersection*`
that road leads to). If that neighbor hasn't been visited yet, mark it
visited, record `current` as its parent, cache its pointer, and enqueue it.
If the `while` loop exhausts the queue without ever hitting the goal check
above, there is no path, and the function returns `{}`.

**Why:** BFS treats every road identically regardless of `length`,
`travel_cost`, or `speed_limit_ms` — it only asks "is this road closed or
not." That's the defining characteristic of this strategy: it finds the path
with the *fewest roads/edges*, which is a completely different notion of
"shortest" than Dijkstra's or A*'s cost-weighted shortest path. On a real
road network where roads have wildly different lengths and speed limits,
BFS's answer can be a genuinely worse route in both distance and time — it
just happens to cross the fewest intersections. `road->is_closed()` being
checked here, identically to how `Dijkstra` and `AStar` check it, is what
lets a closed road (e.g. from a live map edit) actually remove itself from
every algorithm's consideration without any of the three needing special
knowledge of *why* it's closed.

Marking a node `visited` at *discovery* time (when it's found as a neighbor),
not at *expansion* time (when it's popped), is the standard BFS optimization
that prevents the same node from being enqueued multiple times through
different paths before it's ever dequeued — without it, the frontier could
grow unboundedly on a densely connected grid.

**Usage:** This is the "unweighted" half of the Strategy Pattern's payoff:
`BFS` ignores `CostMode` entirely (it never calls `road->cost(...)` or reads
`cost_mode_` anywhere in this file) even though it inherits `set_cost_mode`
from `PathFinding` and a caller can legally call it — the call would just be
a no-op as far as BFS's actual search behavior goes. That's worth flagging
plainly: it's not a bug, but it is a slightly leaky abstraction — the
interface promises cost-mode control that one of the three concrete
strategies silently ignores.

---

```cpp
std::vector<Intersection*> BFS::reconstruct_path(const std::string& start_id,
                                          const std::string& end_id) {
    // node_of was filled during expansion, so the path is just a walk back.
    // The old version scanned the whole prev map and every parent's outgoing
    // edges to rebuild information it was already holding.
    std::vector<Intersection*> path;
    std::string cur = end_id;

    while (true) {
        auto it = node_of.find(cur);
        if (it == node_of.end())
            return {};                  // should never happen on a valid call
        path.push_back(it->second);

        if (cur == start_id)
            break;

        auto pit = prev.find(cur);
        if (pit == prev.end() || pit->second == nullptr)
            return {};
        cur = pit->second->name;
    }

    std::reverse(path.begin(), path.end());
    return path;
}
```

**Meaning:** Starting from `end_id`, repeatedly look up the current node's
pointer in `node_of` and append it to `path`, then step to its parent via
`prev` (by name, since `prev` is keyed on `std::string`). The loop stops when
it reaches `start_id`. Because the walk goes backward (goal to start), the
accumulated `path` is in reverse order, so `std::reverse` fixes it before
returning.

**Why:** The comment is explicit about a real refactor: an earlier version
apparently reconstructed paths by re-scanning `prev` and re-walking each
parent's `outgoing_edges` to find the right `Road`/`Intersection` again —
work `find_path` had already done once during the search. Caching pointers
directly in `node_of` as they're discovered turns reconstruction into a
plain O(path length) walk with no redundant graph lookups. This is a good,
concrete example of "why" a data structure choice was made: it's not
theoretical, it's a documented before/after.

**Usage:** `reconstruct_path` is only ever called from inside `find_path`,
once, at the goal check — it's a private implementation detail split out
purely for readability, not because anything else needs to call it.

**Check yourself:**
- Why is `is_closed()` checked when expanding edges, but road `length` or
  `travel_cost` never consulted anywhere in this file?
- Suppose two different roads both lead to the same never-before-seen
  intersection from two different nodes currently in the queue. Which one
  "wins" as that intersection's `prev`, and why does the order of iteration
  over `outgoing_edges` combined with when `visited` gets marked settle
  that?
- What does calling `set_cost_mode(CostMode::Time)` on a `BFS` object
  actually change about its behavior?

---

### `cpp_backend/include/routing/Dijkstra.hpp`

The second strategy: cost-weighted shortest path, correct for any
non-negative edge weights.

```cpp
#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <queue>
#include "routing/PathFinding.hpp"
class TrafficGraph;
class Intersection;
class Road;

class Dijkstra final : public PathFinding {
public:
    std::vector<Intersection*> find_path(const TrafficGraph& graph,
                                 const std::string& start_id,
                                 const std::string& end_id) override;

private:
    std::unordered_map<std::string, double> dist;
    std::unordered_map<std::string, Intersection*> prev;
    std::unordered_map<std::string, Intersection*> node_of;
    using PQEntry = std::pair<double, std::string>;
    std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<PQEntry>> pq;
    void init_distances(const TrafficGraph& graph, const std::string& start_id);
    void relax_edge(Intersection* current, Road* road);
    std::vector<Intersection*> reconstruct_path(const std::string& start_id,
                                         const std::string& end_id);
};
```

**Meaning:** Same shape as `BFS` — `final`, overrides `find_path` — but the
scratch state is different: `dist` maps an id to its current best-known cost
(a `double`, not present in BFS at all), `prev`/`node_of` serve the same role
as in `BFS`. `PQEntry` is a type alias for `std::pair<double, std::string>` —
(cost, id). `pq` is a `std::priority_queue<PQEntry, std::vector<PQEntry>,
std::greater<PQEntry>>`: the three template arguments are (element type,
underlying container, comparator). Two new private methods,
`init_distances` and `relax_edge`, split out pieces of Dijkstra's algorithm
that `BFS` didn't need at all.

**Why:** `std::priority_queue`'s *default* comparator is `std::less`, which
makes it a **max-heap** — `.top()` gives you the *largest* element. Dijkstra
needs the opposite: at every step, expand the *cheapest* unsettled node
first. `std::greater<PQEntry>` inverts the ordering so `.top()` gives the
*smallest* `(cost, id)` pair — a **min-heap**. `std::pair`'s default
`operator<`/`operator>` compares lexicographically (first by `.first`, the
`double` cost, then by `.second`, the id, only to break exact ties), which is
exactly the "lowest cost first, tie-break by id" ordering Dijkstra wants. A
`priority_queue` (binary heap under the hood, backed here by
`std::vector<PQEntry>`) gives O(log n) push and O(log n) pop-the-min, which
is what makes Dijkstra's classic O((V + E) log V) complexity bound possible
— a linear scan for the minimum every iteration (no heap at all) would be
O(V²), fine for small maps but quadratic on a large one.

**Usage:** This `PQEntry`/`std::greater` pattern reappears identically in
`AStar.hpp` (with `f_score` instead of `dist` as the ordering key) — it's the
shared machinery every cost-aware strategy in this codebase uses. `BFS`
needs none of it, which is the clearest structural signal of how the three
strategies actually differ: **queue** (BFS) vs. **priority-queue** (Dijkstra,
AStar).

**Check yourself:**
- Why does `std::priority_queue<PQEntry, std::vector<PQEntry>,
  std::greater<PQEntry>>` need `std::greater` spelled out explicitly, when
  `std::priority_queue<T>` alone would compile just fine?
- `dist` is `std::unordered_map<std::string, double>`, not
  `std::unordered_map<std::string, std::optional<double>>` or a dense
  `std::vector<double>` indexed by some integer id. What tradeoff does that
  choice make, and how does the `.cpp` file lean on it (see "Lazy init"
  below)?

---

### `cpp_backend/src/routing/Dijkstra.cpp`

```cpp
std::vector<Intersection*> Dijkstra::find_path(const TrafficGraph& graph,
                                        const std::string& start_id,
                                        const std::string& end_id) {
    // clear state 
    dist.clear();
    prev.clear();
    node_of.clear();
    while (!pq.empty()) pq.pop();
    nodes_explored_ = 0;
```

**Meaning:** Same reset pattern as `BFS::find_path`, adapted to Dijkstra's
containers: `dist` instead of `visited`, and `pq` drained the same
manual-`while`-loop way `front` was in BFS (`std::priority_queue` has no
`clear()` either).

**Why/Usage:** Identical rationale to the equivalent BFS block above — reused
member scratch state must be reset explicitly at the top of every call.

---

```cpp
    // validate that both nodes exist in the graph
    if (graph.intersections.find(start_id) == graph.intersections.end() ||
        graph.intersections.find(end_id)   == graph.intersections.end())
        return {};

    // Check if the start and end are the same node
    if (start_id == end_id)
        return { graph.intersections.at(start_id).get() };

    init_distances(graph, start_id);
```

**Meaning:** The same existence check as `BFS`, then a new early return BFS
didn't have: if start and end are literally the same intersection, return a
single-element path immediately (`graph.intersections.at(start_id)` — `.at`
rather than `.find`/`operator[]`, which is safe here because existence was
already confirmed above, and `.at` documents "this must exist" more clearly
than `operator[]`, which would additionally risk *inserting* a default entry
into `intersections` if it somehow didn't). Then `init_distances` seeds the
search from `start_id`.

**Why:** BFS's loop structure happens to handle the start==end case
correctly for free (the goal check fires on the very first pop). Dijkstra's
loop, as you'll see, has a "skip stale/already-settled entries" check that
would also handle it correctly — so this early return is arguably a small
optimization/clarity choice rather than a strict necessity, but it does save
a full `init_distances` + one loop iteration for a trivial query, and it
makes the "same node" case explicit and easy to spot when reading the
function top to bottom.

**Usage:** `AStar::find_path` has the exact same `start_id == end_id` early
return — another example of shared convention across the cost-aware
strategies that isn't enforced by the base class, just followed by
discipline.

---

```cpp
void Dijkstra::init_distances(const TrafficGraph& graph,
                               const std::string& start_id) {
    // Lazy init: "absent from dist" already means infinity, so there is no
    // reason to write 2617 infinities before a search that settles a few
    // hundred nodes.
    Intersection* start = graph.intersections.at(start_id).get();
    dist[start_id] = 0.0;
    prev[start_id] = nullptr;
    node_of[start_id] = start;
    pq.push({ 0.0, start_id });
}
```

**Meaning:** Only the start node gets an explicit entry: `dist[start_id] =
0.0` (cost to reach itself is zero), `prev[start_id] = nullptr` (no parent,
same sentinel convention as BFS), `node_of[start_id]` caches the pointer, and
`{0.0, start_id}` seeds the priority queue.

**Why:** The comment names the design choice directly: classic textbook
Dijkstra initializes *every* node's distance to infinity up front, then
relaxes it down. This codebase instead treats "not present as a key in
`dist`" as an implicit infinity (see `relax_edge` below, which checks
`dist.find(...) == dist.end()`), and only ever inserts an entry once a node
has actually been reached. `2617` in the comment is a specific, concrete
number — presumably the intersection count on some real map this project
tested against — which is a nice, honest way of saying "we measured this and
it mattered," not a hypothetical justification. On a route between two
nearby points in a much larger city-scale map, this lazy approach touches
only the handful of nodes the search actually explores, not the whole graph.

**Usage:** `AStar::init_scores` follows the identical lazy-init idiom for
`g_score`, with the same comment verbatim — confirming this is a codebase-
wide convention for `unordered_map`-backed cost tables, not a one-off choice.

---

```cpp
    while (!pq.empty()) {
        auto [cost, current_name] = pq.top();
        pq.pop();

        // skip relaxed nodes
        if (cost > dist[current_name])
            continue;

        ++nodes_explored_;   // settled: this node's distance is now final
```

**Meaning:** `auto [cost, current_name] = pq.top();` is structured binding
(C++17) unpacking the `PQEntry` pair into two named locals in one line. The
stale-entry check: if the cost stored in this popped queue entry is *greater*
than the current best-known distance for that node (`dist[current_name]`),
this entry is an outdated duplicate — skip it without counting it as
explored.

**Why:** This is the standard "lazy deletion" technique for using
`std::priority_queue` (which has no `decrease-key` operation) to implement
Dijkstra. When a shorter path to a node is found after that node is already
in the queue, `relax_edge` (below) just pushes a *new*, cheaper entry rather
than trying to modify the old one in place — so the same node id can appear
in `pq` multiple times with different costs. When the *stale*, more
expensive copy is eventually popped, this check catches it and `continue`s
past it instead of re-processing an already-settled node. Note this
`dist[current_name]` read uses `operator[]`, not `.find` — at this point
`current_name` is guaranteed present in `dist` (it can only be in the queue
because `relax_edge` or `init_distances` put an entry for it there first),
so the accidental-insertion risk of `operator[]` is moot here, unlike the
`.find`-based checks used to test *absence* elsewhere in this file.

**Usage:** This exact pattern — pop, compare against the authoritative table,
skip if stale, otherwise increment `nodes_explored_` — reappears essentially
verbatim in `AStar::find_path` with `f_score` in place of `dist`. It's the
mechanical core that makes both algorithms' `nodes_explored_` count mean
"nodes actually settled," honoring the contract documented in
`PathFinding.hpp`.

---

```cpp
        // Exit early if we can find a shortest path
        if (current_name == end_id)
            return reconstruct_path(start_id, end_id);

        Intersection* current = graph.intersections.at(current_name).get();

        // Relax all outgoing edges
        for (Road* road : current->outgoing_edges) {
            if (road->is_closed()) continue;   // closed road is untraversable
            relax_edge(current, road);
        }
    }

    // No path exists
    return {};
}
```

**Meaning:** If the just-popped, non-stale node is the goal, Dijkstra can
stop immediately — because it's a min-heap, the first time the goal is
*settled* (not merely discovered), that settlement cost is provably the
shortest possible. Otherwise, look up the live `Intersection*` and relax
every non-closed outgoing road.

**Why:** This early-exit-on-settling-the-goal is exactly what a correct
Dijkstra implementation is allowed to do, and it's *why* Dijkstra doesn't
need to keep running until the whole graph is explored — once the goal is
popped as the minimum-cost entry in the heap, no future pop can ever produce
a cheaper path to it, because all remaining queue entries have cost ≥ the
one just popped.

**Usage:** Compare this early exit to BFS's identically-placed one — same
shape, same location in the loop, but the *guarantee* it relies on is
different: BFS's is "fewest edges because FIFO order," Dijkstra's is
"lowest total cost because min-heap order." This is the clearest place to
see that BFS and Dijkstra solve genuinely different problems that happen to
share a code skeleton.

---

```cpp
void Dijkstra::relax_edge(Intersection* current, Road* road) {
    Intersection* neighbour = road->target;
    double new_dist = dist[current->name] + road->cost(cost_mode_);

    // find() rather than operator[]: an id we have never reached is absent,
    // which is exactly "distance infinity".
    auto it = dist.find(neighbour->name);
    if (it == dist.end() || new_dist < it->second) {
        dist[neighbour->name] = new_dist;
        prev[neighbour->name] = current;
        node_of[neighbour->name] = neighbour;
        pq.push({ new_dist, neighbour->name });
    }
}
```

**Meaning:** "Relaxing" an edge: compute the cost of reaching `neighbour` via
`current` (`current`'s known best distance plus this road's cost, from
`road->cost(cost_mode_)` — the exact call site where `PathFinding`'s
`cost_mode_` finally does something, unlike in `BFS`). Then check: has
`neighbour` never been reached before (`it == dist.end()`), or is this new
path strictly cheaper than its previously known best (`new_dist <
it->second`)? If either, update `dist`, `prev`, `node_of`, and push a fresh
`(new_dist, neighbour->name)` entry onto the priority queue.

**Why:** `road->cost(cost_mode_)` is the single line that makes Dijkstra
*cost-aware* in a way BFS structurally cannot be — `cost_mode_` (inherited
from `PathFinding`, settable via `set_cost_mode`) decides whether that call
returns `length` (`CostMode::Distance`) or `travel_time_seconds()`
(`CostMode::Time`, the default), per `Road::cost()`'s own implementation in
`Road.cpp`. Pushing a new queue entry rather than mutating an existing one
(as noted above) is what avoids needing `std::priority_queue`'s missing
decrease-key operation, at the cost of the queue potentially holding several
stale duplicate entries for the same node — a small extra memory/log-n cost
that's cheap next to the alternative of implementing a custom indexed heap.

**Usage:** This is the method where a closed road stops mattering "for free"
— `find_path`'s loop already filters `road->is_closed()` before ever calling
`relax_edge`, so `relax_edge` itself doesn't need to re-check it. If a road's
`is_closed_` flag flips between calls (a live map edit), the next
`find_path` call simply sees a different `outgoing_edges` traversal outcome
— no cached state in `Dijkstra` itself needs to be invalidated, because
everything is recomputed from scratch every call (that's exactly what the
top-of-function reset block buys).

---

```cpp
std::vector<Intersection*> Dijkstra::reconstruct_path(const std::string& start_id,
                                               const std::string& end_id) {
    // node_of was filled during relaxation, so the path is just a walk back.
    ...
}
```

**Meaning/Why/Usage:** Byte-for-byte the same algorithm as
`BFS::reconstruct_path` (walk backward via `prev`, collecting from
`node_of`, then `std::reverse`), just relying on `prev`/`node_of` as
populated by `relax_edge` instead of BFS's neighbor-expansion loop. The
comment about the "old version" scanning `prev` and re-walking
`outgoing_edges` applies identically here — this refactor clearly touched
all three algorithms' reconstruction logic together.

**Check yourself:**
- Why is a stale, already-superseded entry allowed to sit in `pq` at all,
  rather than removed the moment a cheaper path to the same node is found?
  What does the `if (cost > dist[current_name]) continue;` check cost in
  exchange for allowing that?
- `relax_edge` checks `new_dist < it->second` (strictly less than) to decide
  whether to update. What would change, in terms of correctness or just
  wasted work, if it used `<=` instead?
- If every road's `travel_cost` were forced to equal exactly `1.0`
  regardless of `cost_mode_`, would `Dijkstra::find_path` and
  `BFS::find_path` return the same route for every query? Why or why not?

---

### `cpp_backend/include/routing/AStar.hpp`

The third strategy: Dijkstra plus a heuristic that steers the search toward
the goal instead of expanding uniformly outward in every direction.

```cpp
#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <queue>
#include "routing/PathFinding.hpp"

class TrafficGraph;
class Intersection;
class Road;

class AStar final : public PathFinding {
public:
    std::vector<Intersection*> find_path(const TrafficGraph& graph,
                                 const std::string& start_id,
                                 const std::string& end_id) override;

private:
    std::unordered_map<std::string, double> g_score;
    std::unordered_map<std::string, double> f_score;
    std::unordered_map<std::string, Intersection*> prev;
    std::unordered_map<std::string, Intersection*> node_of;
    using PQEntry = std::pair<double, std::string>;
    std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<PQEntry>> pq;

    void init_scores(const TrafficGraph& graph, const std::string& start_id,
                     const std::string& end_id);
    double heuristic(Intersection* a, Intersection* b);
    double max_speed_ms_ = 0.0;   // fastest speed limit on the map, set per query in Time mode
    std::vector<Intersection*> reconstruct_path(const std::string& start_id,
                                         const std::string& end_id);
};
```

**Meaning:** Structurally almost identical to `Dijkstra`, but `dist` is
replaced by *two* tables: `g_score` (cost from `start` to this node — exactly
what Dijkstra called `dist`) and `f_score` (`g_score` plus the estimated
remaining cost to the goal — the value actually used to order the priority
queue). A new private method, `heuristic`, and a new private member,
`max_speed_ms_`, appear — neither has an equivalent in `BFS` or `Dijkstra`.

**Why:** This is the textbook A* distinction spelled out in the member list
itself: Dijkstra orders its queue purely by cost-so-far (`dist`/`g`); A*
orders its queue by cost-so-far *plus a forward-looking estimate*
(`f = g + h`), so nodes that are heuristically "in the right direction"
toward the goal get explored before equally-cheap-so-far nodes that are
heuristically further away. `max_speed_ms_` exists specifically to make the
heuristic admissible when `cost_mode_ == CostMode::Time` — you'll see exactly
how in the `.cpp` file.

**Usage:** This is the header-level proof of the Strategy Pattern's payoff
promised in `PathFinding.hpp`'s `nodes_explored()` comment: BFS uses a
`std::queue` (FIFO, no ordering by cost at all), Dijkstra and AStar both use
a `std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<PQEntry>>`
(same container, same comparator), but AStar's queue key (`f_score`, cost
plus heuristic) differs from Dijkstra's (`dist`, cost alone) — that one
difference is the entire mechanism by which AStar is expected to settle
fewer nodes than Dijkstra for the same query while still returning the same
optimal route.

**Check yourself:**
- `g_score` here plays exactly the role `dist` played in `Dijkstra`. Why
  give it a different name instead of reusing `dist` for consistency?
- What would happen to AStar's search order — not its final answer, just how
  many nodes it explores before finding it — if `heuristic()` always
  returned `0.0` for every pair of nodes?

---

### `cpp_backend/src/routing/AStar.cpp`

```cpp
// Euclidean (straight-line) distance heuristic.
//
// Admissibility: h(a, b) must never overestimate the true cost from a to the
// goal, which is what guarantees A* returns an optimal path. This holds when
// every edge's traversal cost (travel_cost) is at least the Euclidean distance
// between its endpoints -- the normal case for real roads, since a road cannot
// be shorter than the straight line joining its ends and travel_cost >=
// base_weight = length. By the triangle inequality, the straight-line distance
// from any node to the goal is then a lower bound on the remaining path cost.
//
// Skipping closed roads only removes edges from the graph, which can only
// raise the true shortest-path cost; the heuristic is unchanged and therefore
// remains admissible.
double AStar::heuristic(Intersection* a, Intersection* b) {
    double dx = a->x - b->x;
    double dy = a->y - b->y;
    double h = std::sqrt(dx * dx + dy * dy);
    // For Time mode, the admissible bound is "shortest possible time":
    // straight-line distance over the fastest speed any road allows. Without
    // this, h can overestimate and A* stops being optimal. Distance mode
    // keeps the plain Euclidean distance.
    if (cost_mode_ == CostMode::Time && max_speed_ms_ > 0.0)
        h /= max_speed_ms_;
    return h;
}
```

**Meaning:** `a->x`, `a->y`, `b->x`, `b->y` are the `double` map coordinates
on `Intersection` (from Module 02). `h = sqrt(dx² + dy²)` is the ordinary
Euclidean distance formula. In `CostMode::Distance`, `h` is returned as-is —
straight-line distance in map units. In `CostMode::Time`, `h` is additionally
divided by `max_speed_ms_` (the fastest speed limit found anywhere on the
current map, in meters/second — see `speed_limit_ms()` in `Road.hpp`,
`speed_limit_ / 3.6`, converting km/h to m/s), converting a distance
estimate into a *time* estimate: "the least time it could possibly take to
cover this straight-line distance, if you could travel it at the single
fastest speed limit anywhere on the map."

**Why:** This is the direct answer to the assignment's central question —
what heuristic does AStar use, and is it admissible/consistent for this
graph? The answer, straight from the comment, is: yes, by construction, and
the comment doesn't just assert it, it proves it. The chain of reasoning:
(1) every road's cost is at least its `base_weight` (`length`, i.e. a
straight-line-or-longer measurement — see `Road.hpp`'s comment "Travel cost
of this road in ideal conditions"), so no real road can be shorter than the
straight line between its endpoints; (2) `travel_cost >= base_weight` — see
`Road::update_travel_cost()`, which only ever *increases* cost above the
ideal-conditions floor as congestion rises; (3) by the triangle inequality,
the straight-line distance from any node to the goal can never exceed the
true cheapest path cost through the actual road network, since roads are
themselves at least as long as their straight-line span, and any multi-hop
path is at least as long as the direct line between its endpoints; hence `h`
is always a lower bound (never an overestimate), which is the textbook
definition of admissibility. The Time-mode division by `max_speed_ms_` is
the same argument transplanted onto time: the fastest anything could
possibly cross a straight-line span, in this simulator, is at the single
fastest speed limit on the entire map, so `distance / max_speed_ms_` is
still a lower bound on true travel time, preserving admissibility in that
mode too. The comment's closing paragraph closes a real gap you might
otherwise ask about: does filtering out closed roads break this guarantee?
No — removing edges can only make the *true* shortest path more expensive
(or nonexistent), never cheaper, and the heuristic doesn't reference roads at
all (only straight-line coordinates), so it stays a valid lower bound
regardless of which roads happen to be closed at query time.

Whether this heuristic is also *consistent* (a stronger property than
admissible: `h(n) <= cost(n, n') + h(n')` for every edge `n -> n'`) isn't
spelled out explicitly in the comment, but it follows from the same
triangle-inequality argument — the straight-line distance from `n` to the
goal can never exceed the straight-line distance from `n` to `n'` plus the
straight-line distance from `n'` to the goal, and each edge's actual cost is
at least its own straight-line span, so `h(n) <= dist(n,n') + h(n') <=
cost(n,n') + h(n')`. Consistency is the property that matters in practice
here: it's what guarantees no node is ever "re-opened" after being settled —
the moment a node is popped from `pq` in the loop below with `f_score` equal
to the value it was pushed with, that `f_score` is guaranteed final, which
is exactly the "skip stale entries" pattern this codebase leans on
throughout.

**Usage:** `max_speed_ms_` is recomputed *once per query*, not cached across
queries (see the next block) — specifically so that a live map edit (a speed
limit changed, a fast road added) is picked up by the very next `find_path`
call without needing any explicit invalidation logic, at a cost the comment
elsewhere calls "negligible next to the search itself."

---

```cpp
std::vector<Intersection*> AStar::find_path(const TrafficGraph& graph,
                                        const std::string& start_id,
                                        const std::string& end_id) {
    // clear state
    g_score.clear();
    f_score.clear();
    prev.clear();
    node_of.clear();
    while (!pq.empty()) pq.pop();
    nodes_explored_ = 0;

    // validate that both nodes exist in the graph
    if (graph.intersections.find(start_id) == graph.intersections.end() ||
        graph.intersections.find(end_id)   == graph.intersections.end())
        return {};

    if (start_id == end_id)
        return { graph.intersections.at(start_id).get() };
```

**Meaning/Why:** Identical reset-then-validate opening to `Dijkstra`'s, just
with two score tables cleared instead of one `dist` table. Same rationale:
reused member state must not leak between calls, and the trivial
same-node case is handled without running the search machinery.

---

```cpp
    // In Time mode the admissible heuristic is distance / fastest speed on
    // the map. Recompute the max speed once per query so it stays fresh after
    // live map edits; a tight no-allocation loop is negligible next to the
    // search itself.
    if (cost_mode_ == CostMode::Time) {
        max_speed_ms_ = 0.0;
        for (const auto& [id, road] : graph.edges)
            if (road) max_speed_ms_ = std::max(max_speed_ms_, road->speed_limit_ms());
    }

    init_scores(graph, start_id, end_id);
    Intersection* goal = graph.intersections.at(end_id).get();
```

**Meaning:** `graph.edges` is `TrafficGraph`'s `std::unordered_map<std::string,
std::unique_ptr<Road>>` (Module 02). The loop uses a structured binding
`[id, road]` to iterate (key, value) pairs, checks `if (road)` (guarding
against a null `unique_ptr`, though whether that ever actually occurs
depends on how `edges` gets populated — a defensive check regardless), and
tracks the maximum `speed_limit_ms()` seen across every road on the map into
`max_speed_ms_`, only bothering to do this scan at all when `cost_mode_ ==
CostMode::Time` (in `Distance` mode `max_speed_ms_` is simply never used by
`heuristic()`, so recomputing it would be wasted work). `init_scores` seeds
`g_score`/`f_score`/`prev`/`node_of`/`pq` for the start node, and `goal` is
resolved once, up front, since every call to `heuristic()` in the loop below
needs it.

**Why:** This is the direct implementation of the comment: rather than cache
`max_speed_ms_` as a value computed once when the map loads (which would go
stale the instant a road's speed limit changes via a live map edit — see
`learn/08-supporting-systems.md` for `MapEditor`), it's recomputed fresh at
the top of every single `find_path` call. That trades a full O(E) scan over
every road per query for correctness-by-construction: there's no cache to
remember to invalidate. Given routing queries are not the innermost hot loop
of the simulator (vehicle movement each tick is), that tradeoff favors
simplicity over the last bit of performance.

**Usage:** This is the one place `AStar` reaches past the single `graph`
parameter's `intersections` map into its `edges` map — neither `BFS` nor
`Dijkstra` ever touches `graph.edges` directly; both only ever reach roads
indirectly through `Intersection::outgoing_edges`. That's a direct
consequence of AStar's heuristic needing global map information (the fastest
speed limit *anywhere*), where BFS and Dijkstra's algorithms are purely
local (only ever look at the current node's own outgoing roads).

---

```cpp
    while (!pq.empty()) {
        auto [f, current_name] = pq.top();
        pq.pop();

        // skip stale entries
        if (f > f_score[current_name])
            continue;

        ++nodes_explored_;   // settled: this node's distance is now final

        // exit early when goal is reached
        if (current_name == end_id)
            return reconstruct_path(start_id, end_id);

        Intersection* current = graph.intersections.at(current_name).get();
```

**Meaning/Why:** Structurally identical to `Dijkstra`'s main loop — pop,
compare against the authoritative table (`f_score` here instead of `dist`),
skip stale duplicates, count as explored, check for goal, resolve the live
pointer. Because `f_score = g_score + heuristic(...)` and `heuristic` is
admissible (proved above), the same "first time the goal is popped, its cost
is optimal" argument that justified Dijkstra's early exit still holds for
A* — that's precisely what admissibility buys you: A*'s early exit is as
safe as Dijkstra's, while its ordering explores fewer irrelevant nodes to
get there.

---

```cpp
        for (Road* road : current->outgoing_edges) {
            if (road->is_closed()) continue;   // closed road is untraversable
            Intersection* neighbour = road->target;
            double tentative_g = g_score[current_name] + road->cost(cost_mode_);

            // find() rather than operator[]: an id we have never reached is
            // absent from g_score, which is exactly "cost so far infinity".
            auto it = g_score.find(neighbour->name);
            if (it == g_score.end() || tentative_g < it->second) {
                prev[neighbour->name] = current;
                node_of[neighbour->name] = neighbour;
                g_score[neighbour->name] = tentative_g;
                f_score[neighbour->name] = tentative_g + heuristic(neighbour, goal);
                pq.push({ f_score[neighbour->name], neighbour->name });
            }
        }
    }

    return {};
}
```

**Meaning:** This is `Dijkstra::relax_edge` with one addition: when a
cheaper path to `neighbour` is found, in addition to updating `g_score`
(A*'s equivalent of Dijkstra's `dist`), it also computes `f_score[neighbour]
= tentative_g + heuristic(neighbour, goal)` and pushes *that* combined value
onto the queue — not the raw `g_score`. Everything else (the `road->cost(
cost_mode_)` call, the `is_closed()` filter, the `find()`-not-`operator[]`
absence check) is identical in spirit and even in phrasing to `Dijkstra`'s
`relax_edge`.

**Why:** This one line — pushing `f_score` instead of `g_score` as the
ordering key — is the entire algorithmic delta between Dijkstra and A*. Both
guarantee optimality (given non-negative, admissible-heuristic conditions);
A*'s advantage is purely in *which order* it visits nodes while searching
for that optimum, biasing exploration toward the goal instead of expanding a
uniform "distance from start" frontier in every direction the way Dijkstra
effectively does. This method isn't split into its own `relax_edge`-style
private function the way Dijkstra's is — a small, mildly inconsistent
stylistic choice across the two files, but not a functional one.

**Usage:** `reconstruct_path` here is, again, byte-for-byte the same
`node_of`/`prev`/`std::reverse` walk-back as `BFS`'s and `Dijkstra`'s.

**Check yourself:**
- Walk through the admissibility argument in your own words: why does
  `travel_cost >= base_weight = length` combined with "a road can't be
  shorter than the straight line between its endpoints" guarantee `h` never
  overestimates the true remaining cost?
- In `CostMode::Time`, why divide by the single fastest speed limit on the
  *entire* map, rather than, say, the average speed limit, or the speed
  limit of the specific road being considered?
- If a future change made `Road::cost()` return a value that could be
  *negative* (e.g. some kind of routing incentive/discount), which of BFS,
  Dijkstra, and AStar's correctness guarantees would break first, and why?
- Both `Dijkstra` and `AStar` use `std::priority_queue<PQEntry,
  std::vector<PQEntry>, std::greater<PQEntry>>` with the exact same
  `PQEntry` shape. What is the *only* semantic difference in how each fills
  that queue, and where exactly in each `.cpp` file does that difference
  appear?

---

## The Strategy Pattern, tied together

You've now seen the interface (`PathFinding`) and all three concrete
strategies. The last piece is where the pattern actually pays for itself: the
caller. `Vehicle` (covered in full in
[`learn/05-vehicles.md`](05-vehicles.md)) owns the strategy through a
`std::unique_ptr<PathFinding> router_;` — a pointer to the *base* class, not
to any concrete one. `Vehicle::run_router` calls through it exactly once:

```cpp
std::vector<Intersection*> Vehicle::run_router(const TrafficGraph& graph,
                                               const std::string& start_id,
                                               const std::string& target_id) const {
    return router_->find_path(graph, start_id, target_id);
}
```

That single line — `router_->find_path(...)` — never changes no matter which
concrete strategy `router_` currently points to. Swapping the algorithm is
done in exactly one place, `Vehicle::set_algorithm`, in `Vehicle.cpp`:

```cpp
void Vehicle::set_algorithm(const std::string& name) {
    if (name == algorithm_ && router_) return;  // already running this one
    algorithm_ = name;
    router_ = make_router(name);
}
```

`make_router` (a free function local to `Vehicle.cpp`) is the one place in
the whole codebase — per its own comment — allowed to name `BFS`,
`Dijkstra`, and `AStar` directly:

```cpp
std::unique_ptr<PathFinding> make_router(const std::string& name) {
    if (name == "astar") return std::make_unique<AStar>();
    if (name == "bfs")   return std::make_unique<BFS>();
    return std::make_unique<Dijkstra>();
}
```

This is the textbook shape of the Strategy Pattern end to end: a `Vehicle`
never has an `if (algorithm_ == "astar") ... else if (algorithm_ == "bfs")
...` branch anywhere near its actual routing call — that branching happens
exactly once, at construction time inside `make_router`, and everywhere else
(`run_router`, `set_cost_mode`, the destructor when a vehicle is removed) the
code is written against `PathFinding` alone. An unrecognized algorithm name
silently falls back to `Dijkstra` (the `return std::make_unique<Dijkstra>();`
at the end, reached whenever neither `"astar"` nor `"bfs"` matched) —
`Dijkstra` is a defensible fallback since it's the only one of the three that
is both correct-by-cost and doesn't depend on map coordinates being
meaningful the way `AStar`'s heuristic does.

Practically, given what you've now read in all three `.cpp` files: for the
same start/end pair and the same `CostMode`, `Dijkstra` and `AStar` return
the *same route* (both are cost-optimal) but `AStar` should report a lower
`nodes_explored()` because its heuristic steers search toward the goal
instead of expanding uniformly in every direction. `BFS` is the cheapest per
node explored (no `double` arithmetic, no heap, just a FIFO queue) but is the
only one of the three whose route is not generally cost-optimal — it
optimizes edge count, not distance or time, which will visibly diverge from
the other two on any map with roads of unequal length or speed limit.

**Check yourself:**
- If a new caller wanted to add a fourth algorithm (say, bidirectional
  search), which files would it need to touch, and which files — critically
  — would it *not* need to touch?
- `make_router` is described as "the only place in the codebase that names
  Dijkstra, AStar or BFS directly." Why does keeping that true matter for
  how easy it is to add a fifth algorithm later?
- Why does an unrecognized algorithm name fall back to `Dijkstra` specifically,
  rather than to `BFS` (cheapest) or `AStar` (fewest nodes explored)?
