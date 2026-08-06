# Module 06 — Events and Observer Wiring

**Files covered, in order:**
1. [`cpp_backend/include/utils/Observer.hpp`](../cpp_backend/include/utils/Observer.hpp) / [`cpp_backend/src/utils/Observer.cpp`](../cpp_backend/src/utils/Observer.cpp)
2. [`cpp_backend/include/events/TrafficEvent.hpp`](../cpp_backend/include/events/TrafficEvent.hpp) / [`cpp_backend/src/events/TrafficEvent.cpp`](../cpp_backend/src/events/TrafficEvent.cpp)
3. [`cpp_backend/include/simulation/TrafficController.hpp`](../cpp_backend/include/simulation/TrafficController.hpp) / [`cpp_backend/src/simulation/TrafficController.cpp`](../cpp_backend/src/simulation/TrafficController.cpp)

This module is where the codebase's second design pattern lives (the first
was the Strategy Pattern in routing, see [`learn/04-routing.md`](04-routing.md)
if you've covered it, or `learn/05-vehicles.md` for how vehicles consume the
result). Here it's the **Observer Pattern**: something in the simulation
decides "a road just closed" and a set of vehicles need to hear about it
*without the thing that closed the road knowing which vehicles exist, or
even how many there are*. That decoupling is the entire reason this module
exists — the alternative (the code that closes a road walking every vehicle
and calling a method on each) would tie two unrelated concerns — "network
state changed" and "vehicle behavior" — into one function, and would break
every time either side changed independently.

---

## 1. `cpp_backend/include/utils/Observer.hpp`

```cpp
// This is Observer design pattern implementation for traffic simulation.
// It allows vehicles to subscribe to traffic updates and receive notifications about changes in the traffic network, such as road closures, accidents, congestion, and traffic light status changes.
// The Subject class manages a list of observers and broadcasts notifications to them when relevant events occur.
#pragma once

#include <algorithm>
#include <string>
#include <vector>
```

**Meaning:** The file-level comment states the intent up front: vehicles
subscribe, the network broadcasts. `#pragma once` is the usual
include-guard idiom (see `learn/02-core-graph-model.md` for the first time
this pattern showed up, in `Graph.hpp`). The includes are minimal and
purposeful: `<algorithm>` for `std::find`/`std::remove` (used in the
`.cpp`), `<string>` for the notification payload's text fields, `<vector>`
for the observer list.

**Why:** This is a *utility* header (`utils/`), not a `simulation/` or
`events/` header, and that placement is deliberate — the Subject/Observer
machinery here knows nothing about roads, vehicles, or traffic events
specifically. It's generic enough to subscribe/broadcast *anything*. The
domain-specific payload (`TrafficNotification`) and the domain-specific
Subject (`TrafficController`, module section 3 below) are layered on top of
it, in their own subfolders. That's a deliberate separation: the *pattern*
lives in `utils/`, the *application* of the pattern lives in `events/` and
`simulation/`.

**Usage:** Every file that wants to either broadcast or receive traffic
updates includes this header, directly or transitively.

---

### Lines 10–26: `TrafficNotification` — the payload

```cpp
struct TrafficNotification /// Payload describing a change in the traffic network.
{
    enum class Kind {
        RoadClosed,
        RoadReopened,
        Accident,
        AccidentCleared,
        Congestion,
        CongestionCleared,
        TrafficLight,
        TrafficLightRestored
    };
    Kind kind;
    std::string roadId;          ///< affected road ("" if not road-scoped)
    std::string intersectionId;  ///< affected intersection ("" if not applicable)
    std::string description;     ///< human readable text for logs / UI
};
```

**Meaning:** `TrafficNotification` is a plain data `struct` (all members
public by default, no behavior) — the *message* that flows from broadcaster
to listeners. Its first member, `Kind`, is a scoped enum (`enum class`)
listing eight possible notification kinds, in start/end pairs: a road can
close (`RoadClosed`) and later reopen (`RoadReopened`); an accident can
happen (`Accident`) and clear (`AccidentCleared`); congestion can build
(`Congestion`) and clear (`CongestionCleared`); a traffic light can fault
(`TrafficLight`) and be restored (`TrafficLightRestored`). The three
remaining members are optional-by-convention string fields: `roadId` and
`intersectionId` are filled in only when the notification is scoped to that
kind of target (the comments spell out "" as the "not applicable" sentinel
rather than using `std::optional<std::string>`), and `description` is
free-text meant for logs or the UI, not for programmatic branching.

**Why:** Using `enum class Kind` instead of a raw event-type string keeps
listener code exhaustive and typo-proof — a `switch` over `Kind` can be
checked by the compiler for missed cases (you'll see this switch in
`Vehicle::onNotify`, covered below), where a `std::string` comparison like
`if (type == "roadclosed")` would silently do nothing on a typo. Choosing
`""` as the "not applicable" sentinel over `std::optional<std::string>` is
a small, pragmatic tradeoff: it avoids `.value()`/`.has_value()` ceremony at
every call site, at the cost of the type system not forcing callers to
check emptiness — a reader has to know the convention rather than have the
compiler enforce it.

**Usage:** This is the *only* type that crosses the Subject → Observer
boundary. Every event type in `events/TrafficEvent.hpp` (section 2 below)
produces one of these; every listener (`Vehicle::onNotify`, in
`learn/05-vehicles.md`) pattern-matches on `.kind` to decide what to do.
Adding a ninth kind of notification means adding one enumerator here — no
other type in this header needs to change.

**Check yourself:**
- Why is `Kind` an `enum class` and not a plain `enum`? What would change about how you'd have to write `Kind::RoadClosed` if it weren't scoped?
- Why does `TrafficNotification` carry *both* `roadId` and `intersectionId` instead of one generic `targetId` field?

---

### Lines 28–33: `IObserver` — the Observer interface

```cpp
/// Interface implemented by anything that wants traffic updates (vehicles).
class IObserver {
public:
    virtual ~IObserver() = default;
    virtual void onNotify(const TrafficNotification& notification) = 0;
};
```

**Meaning:** `IObserver` is an abstract base class — a pure interface. It
has exactly one method, `onNotify`, taking a `const TrafficNotification&`
and returning nothing, and it's declared `= 0` (pure virtual), meaning
`IObserver` itself can never be instantiated; only a class that overrides
`onNotify` can be. `virtual ~IObserver() = default;` gives it a virtual
destructor — the compiler generates the trivial destructor body, but marks
it `virtual` so that deleting an `IObserver*` that actually points at a
derived object (e.g. a `Vehicle*` stored as `IObserver*`) runs the derived
destructor too, instead of only the base one.

**Why:** This is the textbook Observer-pattern "Observer" role, expressed
as virtual dispatch rather than `std::function` callbacks. The codebase had
a real choice here: it could have made `Subject` hold a
`std::vector<std::function<void(const TrafficNotification&)>>` instead of a
list of `IObserver*`. The interface approach was chosen instead, and it
buys two things a `std::function` list wouldn't give as cleanly: (1) *identity*
— `attach`/`detach` (see below) need to find and remove a *specific*
registration by comparing pointers, which is trivial for `IObserver*` but
awkward for `std::function` (which isn't equality-comparable in general);
(2) *no dangling-lambda-capture risk* — a `Vehicle` implementing `IObserver`
owns its own `onNotify` state directly as member data, rather than a lambda
having to capture `this` and hope the vehicle outlives the subscription (it
still has to, but the failure mode of forgetting to detach is at least
visible as a raw pointer in a list, not a captured pointer buried inside a
`std::function`).

**Usage:** `Vehicle` is the concrete `IObserver` in this codebase — see
`class Vehicle : public IObserver` in `cpp_backend/include/vehicles/Vehicle.hpp`,
covered in [`learn/05-vehicles.md`](05-vehicles.md). Nothing else in the
codebase implements `IObserver` today, but the interface doesn't know or
care that vehicles are the only listener — a future UI-side logger or a
statistics collector could implement `IObserver` and attach to the same
`TrafficController` with zero changes to this header or to `Subject`.

**Check yourself:**
- Why must `~IObserver()` be `virtual` even though its body does nothing?
- What would go wrong (concretely, at the `Subject::detach` call site) if `Subject` stored `std::function<void(const TrafficNotification&)>` instead of `IObserver*`?

---

### Lines 35–44: `Subject` — the Observer subject base class

```cpp
/// Subject base class: keeps a list of observers and broadcasts to them.
class Subject {
private:
    std::vector<IObserver*> observers_; // List of observers (vehicles) followed to traffic updates.
public:
    virtual ~Subject() = default;
    void attach(IObserver* observer); // Adds an observer to the list of observers if it is not already present.
    void detach(IObserver* observer); // Removes an observer from the list of observers if it is present.
    void notifyAll(const TrafficNotification& notification); // Notifies all observers in the list of observers about a change in the traffic network by calling their onNotify method with the provided notification.
};
```

**Meaning:** `Subject` holds a private `std::vector<IObserver*>
observers_` — a flat list of *non-owning* raw pointers (`Subject` never
`new`s or `delete`s an `IObserver`; it just remembers who to call). It
exposes three public operations: `attach` (subscribe), `detach`
(unsubscribe), and `notifyAll` (broadcast to everyone currently
subscribed). Its destructor is `virtual`, for the same reason as
`IObserver`'s: `Subject` is meant to be a base class (`TrafficController`
inherits from it, section 3), so deleting through a base pointer must still
run the derived destructor.

**Why:** This is the "Subject" role of the pattern — note that `Subject`
here is completely generic; it has no idea a `TrafficNotification` is about
roads or accidents, only that it's *some* payload type observers care
about. Storing raw `IObserver*` (not `std::shared_ptr<IObserver>` or
similar) means `Subject` explicitly does **not** own the observers'
lifetime — ownership of a `Vehicle` lives elsewhere (in
`TrafficSimulator`'s vehicle list, see `learn/07-simulation-core.md`), and
`Subject` only holds a *view* into that list. This is a real, sharp
lifetime obligation: if a `Vehicle` is destroyed while still `attach`ed, the
pointer left behind in `observers_` becomes dangling, and the next
`notifyAll` would call a virtual function through a dead pointer — undefined
behavior. The codebase's answer to that risk isn't in this file at all;
it's a *discipline* enforced at every call site that destroys a vehicle
(see Usage below) — always `detach` before the vehicle goes away. This
header gives you no compiler-enforced protection against forgetting that;
it's a convention, and a fragile one, similar in spirit to the "known
limitations" called out for lane-locking and junction overlap.

**Usage:** `TrafficController` is the one and only concrete `Subject` in
this codebase (`class TrafficController : public Subject`, section 3).
`TrafficSimulator::spawn_vehicle` calls `controller_.attach(v)` right after
constructing a new vehicle (`cpp_backend/src/simulation/TrafficSimulator.cpp:431`,
with the inline comment `// Observer: v now hears about events`), and every
path that removes a vehicle — `destroy_vehicle_locked`, the vehicle-count
trim in the tick loop, `MapEditor`'s road-deletion cleanup — calls
`controller_.detach(v)` first. That symmetry (every `attach` has a matching
`detach` on every removal path) is exactly what keeps the dangling-pointer
risk from firing in practice, even though nothing in `Subject` itself
enforces it.

**Check yourself:**
- Why does `Subject` store raw `IObserver*` instead of `std::shared_ptr<IObserver>`? What would ownership look like if it used shared ownership instead?
- If a `Vehicle` were destroyed without calling `detach` first, what would happen the next time `notifyAll` runs?

---

## `cpp_backend/src/utils/Observer.cpp`

```cpp
#include "utils/Observer.hpp"

void Subject::attach(IObserver* observer)
{
    if (observer == nullptr) return;
    if (std::find(observers_.begin(), observers_.end(), observer) == observers_.end()) {
        observers_.push_back(observer);
    }
}
```

**Meaning:** `attach` first guards against a null pointer (silently does
nothing rather than crashing or asserting). Then it uses `std::find` to
linear-scan `observers_` for an existing entry equal to `observer`
(pointer equality — comparing addresses, not comparing `Vehicle` contents).
Only if `observer` isn't already in the list does it `push_back` it.

**Why:** The null check is defensive: nothing in the header's contract
promises callers won't pass `nullptr` (there's no reference-type
alternative used here), so `attach` protects itself rather than trusting
every caller. The duplicate check matters because `notifyAll` (below) calls
`onNotify` once *per entry* in the vector — if the same vehicle were
attached twice by accident (e.g. a bug that calls `attach` on every tick
instead of once at spawn), it would receive every notification twice,
silently corrupting its `needs_reroute_` logic without ever crashing or
logging an error. `std::find` is `O(n)` here rather than using a
`std::set`/`std::unordered_set`, which is a reasonable tradeoff at this
scale (the observer list is bounded by the number of live vehicles in one
simulation, not something being searched in a hot per-frame loop —
`attach`/`detach` only happen at vehicle spawn/despawn, not every tick).

**Usage:** Called exactly once per vehicle's lifetime, at spawn (see
`learn/07-simulation-core.md` for `TrafficSimulator::spawn_vehicle`).

```cpp
void Subject::detach(IObserver* observer)
{
    observers_.erase(std::remove(observers_.begin(), observers_.end(), observer),
                    observers_.end());
}
```

**Meaning:** The classic erase-remove idiom: `std::remove` doesn't actually
shrink the vector — it shuffles all elements *not* equal to `observer` to
the front and returns an iterator to the new logical end, leaving the tail
in an unspecified state; `.erase(...)` then physically removes that tail,
shrinking the vector's `size()`. Net effect: every occurrence of `observer`
(there should be at most one, given `attach`'s duplicate guard) is dropped.

**Why:** `std::remove` alone would leave the vector's `size()` unchanged
and its tail elements in a moved-from/unspecified state — combining it with
`.erase()` is required to actually shrink the container; this is such a
common near-miss in C++ that the combination has its own name
("erase-remove idiom"). Note this function tolerates `observer` not being
present at all — `std::remove` with no match just returns `end()`, and
`.erase(end(), end())` is a no-op — so `detach` on an unattached (or
already-detached) observer is safe to call, which matters because some
call sites (see Usage) call `detach` defensively without first checking
whether `attach` actually succeeded.

**Usage:** Called on every vehicle-removal path in
`TrafficSimulator.cpp` and `MapEditor.cpp` — search for `controller_.detach`
to find them all. This is the "unsubscribe" half of the lifetime discipline
discussed above: get this wrong (skip a `detach` call) and `observers_`
accumulates dangling pointers over the life of a long-running simulation.

```cpp
void Subject::notifyAll(const TrafficNotification& notification)
{
    // Copy: an observer may detach itself while being notified.
    std::vector<IObserver*> snapshot = observers_;
    for (IObserver* observer : snapshot) {
        observer->onNotify(notification);
    }
}
```

**Meaning:** `notifyAll` takes a full **copy** of `observers_` into a local
`snapshot` vector before iterating, then calls `onNotify` on each pointer
in the *copy*, not the original. The inline comment states why directly:
"an observer may detach itself while being notified."

**Why:** This is the single most important line in this file, and it's
solving a real re-entrancy hazard. If `notifyAll` iterated `observers_`
directly (e.g. a range-`for` over the member vector) and, inside some
vehicle's `onNotify`, that vehicle (or the code calling it) triggered a
`detach` — mutating `observers_` — then the iterator being used by the
outer `for` loop would be invalidated mid-iteration. That's undefined
behavior: anything from skipping an observer, to processing one twice, to
an outright crash, depending on standard library implementation details.
Copying first sidesteps the whole class of bug: mutations to the *real*
`observers_` during the loop can't touch `snapshot`, so the loop is safe
even if a listener detaches itself (or another listener) as a side effect
of handling this very notification. The cost is one `O(n)` vector copy per
broadcast — cheap relative to correctness, given how few observers exist at
once and how infrequently `notifyAll` fires (only on event start/end, not
every tick).

**Usage:** `TrafficController::trigger_event`, `expire_events_targeting`,
and `update` (section 3) are the only three call sites of `notifyAll` in
the codebase — each fires it once for a start notification and once for an
end notification, per event, never per tick per vehicle. That's the whole
point of a broadcast subject: `TrafficController` doesn't need to know how
many vehicles exist or which ones care about a given road; it just says "an
event started" once, and lets `Subject` fan it out.

**Check yourself:**
- Trace through what could go wrong if `notifyAll` iterated `observers_` directly instead of a `snapshot` copy, assuming some `onNotify` implementation calls `detach` on itself.
- Why does `attach` guard against duplicates but `detach` doesn't need to guard against "removing something already removed"?

---

## 2. `cpp_backend/include/events/TrafficEvent.hpp`

```cpp
#pragma once
/*
 * TrafficEvent.hpp

 * Dynamic traffic events. Each event knows how to apply itself to the
 * graph, how to undo itself when it expires, and which notification it
 * emits (consumed by vehicles through the Observer pattern).

 * Class hierarchy (inheritance + polymorphism):
     TrafficEvent (abstract)
       ├── CongestionEvent   : extra travel cost on a road
       ├── AccidentEvent     : capacity collapse + heavy congestion
       ├── RoadClosureEvent  : road temporarily disabled
       └── TrafficLightEvent : longer red phase at an intersection
*/

#include <memory>
#include <string>

#include "core/Graph.hpp"
#include "utils/Observer.hpp"
```

**Meaning:** The header comment draws the class hierarchy directly — four
concrete leaf classes, all deriving from one abstract `TrafficEvent`. The
includes pull in `<memory>` (for `std::unique_ptr`, used by the factory at
the bottom of the file), `<string>`, `core/Graph.hpp` (each event mutates a
`TrafficGraph`, see `learn/02-core-graph-model.md`), and `utils/Observer.hpp`
(for `TrafficNotification` — an event needs to *produce* one, even though
it isn't itself a `Subject` or an `IObserver`).

**Why:** This is a second, independent use of runtime polymorphism in the
codebase — compare it to the Strategy Pattern in `routing/` (`PathFinding`
with `BFS`/`Dijkstra`/`AStar` as interchangeable strategies, see
`learn/04-routing.md`). The shapes look similar (an abstract base,
several `final` leaf overrides) but the *intent* differs: Strategy swaps
out *how* one algorithm is computed, all producing the same kind of answer
(a route); this hierarchy instead models *different things that can happen
to the network*, each with its own state, its own apply/expire behavior,
and its own notification. It's closer to a classic "polymorphic event
type" hierarchy than a Strategy — the shared base isn't "pick one
implementation," it's "these four are all instances of the same abstract
concept: something time-limited that changes the graph and tells someone
about it."

**Usage:** `TrafficController` (section 3) is the sole owner of
`TrafficEvent` instances — it holds a
`std::vector<std::unique_ptr<TrafficEvent>> events_` and drives every
event's `apply`/`advance`/`expire` calls. No other class touches
`TrafficEvent` directly.

**Check yourself:**
- Why does `TrafficEvent.hpp` include `utils/Observer.hpp` even though `TrafficEvent` is neither a `Subject` nor an `IObserver`?

---

### Lines 23–53: `TrafficEvent` — the abstract base

```cpp
class TrafficEvent {
public:
    TrafficEvent(std::string id, std::string targetId, double durationSeconds);
    virtual ~TrafficEvent() = default;

    const std::string& id() const { return id_; }
    const std::string& target() const { return targetId_; }
    double remaining() const { return durationSeconds_ - elapsed_; }
    double duration() const { return durationSeconds_; }
    bool expired() const { return elapsed_ >= durationSeconds_; }

    virtual std::string type() const = 0;
    virtual std::string description() const = 0;

    /// Mutate the graph when the event starts.
    virtual void apply(TrafficGraph& graph) = 0;
    /// Restore the graph when the event ends.
    virtual void expire(TrafficGraph& graph) = 0;

    /// Notifications broadcast to observers on start / end.
    virtual TrafficNotification startNotification() const = 0;
    virtual TrafficNotification endNotification() const = 0;

    void advance(double dt) { elapsed_ += dt; }

protected:
    std::string id_;
    std::string targetId_;
    double durationSeconds_;
    double elapsed_ = 0.0;
};
```

**Meaning:** The constructor takes an `id` (a unique event identifier, e.g.
`"E1"`), a `targetId` (the road or intersection this event affects), and a
`durationSeconds` (how long the event lasts). Six inline accessors expose
read-only views of the state: `id()`, `target()`, `remaining()` (time left,
computed as duration minus elapsed — not stored directly), `duration()`,
and `expired()` (true once `elapsed_` has caught up to `durationSeconds_`).
Six methods are pure virtual (`= 0`): `type()` and `description()` (both
return human/machine-readable strings), `apply`/`expire` (the graph
mutation and its undo), and `startNotification`/`endNotification` (the
`TrafficNotification` payloads to broadcast). `advance(double dt)` is the
one non-virtual, concrete mutator: it just adds `dt` to `elapsed_` — every
subclass shares this exact time-bookkeeping logic, so it isn't duplicated
or made virtual. Everything else (`id_`, `targetId_`, `durationSeconds_`,
`elapsed_`) is `protected`, not `private` — subclasses need direct access
(e.g. `targetId_` is used inside every subclass's `apply`/`description`).

**Why:** This is the same "what varies vs. what's shared" split you'll
recognize from `PathFinding` in the routing module: time bookkeeping
(`advance`, `expired`, `remaining`) is identical for every event type, so
it's implemented once, concretely, in the base — no subclass needs to (or
is allowed to) override it. What's genuinely different per event —
*what happens to the graph* and *what gets announced* — is left abstract.
Six pure virtuals is a fairly wide interface for an abstract base; the
alternative (a `std::variant<CongestionEvent, AccidentEvent, ...>` with
free functions dispatching on which alternative is active) would avoid
virtual-call overhead but would need to be extended at every free function
whenever a new event type is added, whereas this design only requires
adding one new subclass. Given events fire rarely (once per random-event
timer tick, at most, per `TrafficController::update`) rather than every
frame, virtual dispatch overhead is not a real concern here.

**Usage:** `TrafficController::update` calls `event->advance(dt)` on every
live event, `event->expired()` to detect when to undo it, and
`event->expire(*graph_)` / `event->apply(*graph_)` at the right moments
(section 3). `TrafficController::trigger_event` and
`expire_events_targeting` call `startNotification()`/`endNotification()` to
get the payload for `notifyAll`.

**Check yourself:**
- Why is `advance(double dt)` non-virtual while `apply`/`expire` are pure virtual, given that all four subclasses need `advance` to behave identically?
- `remaining()` is computed from `durationSeconds_ - elapsed_` rather than stored as its own field that counts down. What bug class does that avoid?

---

### Lines 56–109: the four concrete event types

```cpp
class CongestionEvent final : public TrafficEvent {
public:
    CongestionEvent(std::string id, std::string roadId, double durationSeconds,
                    double intensity = 0.5);
    std::string type() const override { return "congestion"; }
    ...
private:
    double intensity_;
};
```
```cpp
class AccidentEvent final : public TrafficEvent { ... };
class RoadClosureEvent final : public TrafficEvent { ... };
class TrafficLightEvent final : public TrafficEvent {
    ...
private:
    static constexpr double kBrokenGreenScale = 0.2;
};
```

**Meaning:** Four leaf classes, each `final` (none of them can be further
subclassed — the hierarchy is exactly two levels deep, by design) and each
overriding all six pure virtuals from `TrafficEvent`. `CongestionEvent`
adds one extra constructor parameter, `intensity` (defaulting to `0.5`),
and one extra private member, `intensity_`, to remember it.
`TrafficLightEvent` instead adds a `private static constexpr double
kBrokenGreenScale = 0.2` — a compile-time constant shared by all instances
(not per-instance state, unlike `intensity_`), used to shrink a traffic
light's green phase to 20% of normal. `AccidentEvent` and
`RoadClosureEvent` add no extra members at all — their entire behavior is
expressed through what `apply`/`expire` do to the graph, with no
per-instance tuning knob.

**Why:** Marking every leaf `final` is a small but meaningful signal: this
hierarchy is closed by design — the four event kinds are the complete set
the domain currently models, and the compiler will reject any attempt to
derive further, catching an accidental misuse early rather than allowing an
ad-hoc fifth type to sneak in via inheritance instead of going through
`makeEvent` (below). `intensity_` as a constructor parameter with a default
(`0.5`) rather than a hardcoded magic number lets callers tune "how bad" a
congestion event is per-instance, while `kBrokenGreenScale` being `static
constexpr` (not a constructor parameter) reflects a design choice that
*all* broken-light events behave the same way — there was no perceived need
to let callers dial in how broken a light gets.

**Usage:** These four classes are never named directly by `TrafficController`
— it only ever holds `std::unique_ptr<TrafficEvent>` and calls through the
base interface. The only place their concrete names appear is inside
`makeEvent` (the factory function, covered next) and in the corresponding
`.cpp` file. This is the polymorphism payoff: `TrafficController::update`'s
loop over `events_` doesn't need an `if/else` chain checking which
concrete type each event is — every event, regardless of kind, responds to
the same four calls (`advance`, `expired`, `expire`, `endNotification`).

**Check yourself:**
- Why is `intensity_` a constructor parameter but `kBrokenGreenScale` is a `static constexpr` member instead?
- What would break, mechanically, if someone tried to write `class SuperAccidentEvent : public AccidentEvent`?

---

### Line 112–115: the factory function

```cpp
/// Simple factory for events (used by the controller and the HTTP API).
std::unique_ptr<TrafficEvent> makeEvent(const std::string& type,
                                        const std::string& id,
                                        const std::string& targetId,
                                        double durationSeconds);
```

**Meaning:** A free function (not a class method) that takes a `type`
string plus the three constructor arguments common to every event kind,
and returns a `std::unique_ptr<TrafficEvent>` — owning, heap-allocated,
polymorphic. It's declared here at namespace scope, outside any class.

**Why:** This is the Factory Function/Simple Factory pattern, layered on
top of the event hierarchy: it's the one place in the codebase that knows
the mapping from a string like `"congestion"` to a concrete class like
`CongestionEvent`. Returning `std::unique_ptr<TrafficEvent>` rather than a
raw `TrafficEvent*` means ownership transfers unambiguously to the caller —
no question of who's responsible for `delete`ing it, and no leak if an
exception were thrown between allocation and storage. The comment notes
two callers ("the controller and the HTTP API") — meaning event creation
isn't only internal to the simulation; a REST endpoint that lets a client
manually trigger, say, an accident on a specific road goes through this
exact same factory, so client-triggered and randomly-triggered events are
indistinguishable once created.

**Usage:** `TrafficController::trigger_event` (section 3) is the concrete
call site inside this module; see `learn/09-server-and-entrypoint.md` for
the HTTP-side caller once that module is written.

**Check yourself:**
- Why does `makeEvent` return `std::unique_ptr<TrafficEvent>` instead of `TrafficEvent` by value?
- If a fifth event type were added, how many places in `TrafficEvent.hpp`/`.cpp` would need to change, including this factory?

---

## `cpp_backend/src/events/TrafficEvent.cpp`

```cpp
#include "events/TrafficEvent.hpp"

#include <utility>
#include "core/Road.hpp"
#include "core/Intersection.hpp"
#include "core/TrafficLight.hpp"

using Kind = TrafficNotification::Kind;

TrafficEvent::TrafficEvent(std::string id, std::string targetId,
                           double durationSeconds)
    : id_(std::move(id)),
      targetId_(std::move(targetId)),
      durationSeconds_(durationSeconds) {}
```

**Meaning:** `<utility>` brings in `std::move`. `core/Road.hpp`,
`core/Intersection.hpp`, `core/TrafficLight.hpp` are included here (not in
the header) because only the `.cpp` needs their concrete definitions — the
header only forward-declares/uses `TrafficGraph` by reference, so it
doesn't need to know what a `Road` or `Intersection` actually contains. `using
Kind = TrafficNotification::Kind;` is a local alias so every
`startNotification`/`endNotification` body below can write `Kind::Accident`
instead of the fully qualified `TrafficNotification::Kind::Accident`. The
base constructor takes its three string/double parameters by value and
`std::move`s the strings into the member fields — avoiding an extra copy
beyond the one the caller already made when constructing the arguments.

**Why:** Keeping heavyweight includes (`Road.hpp`, `Intersection.hpp`,
`TrafficLight.hpp`) out of the header and in the `.cpp` is a standard
compile-time-hygiene move: anything that includes `TrafficEvent.hpp` (which
is most of the events/simulation code) doesn't pay the compilation cost of
those three headers unless it also needs them directly. The
pass-by-value-then-move constructor idiom is the same one used throughout
this codebase's other value types.

**Usage:** Every subclass constructor (`CongestionEvent`, `AccidentEvent`,
etc.) forwards its `id`/`targetId`/`durationSeconds` straight into this
base constructor via its member-init list.

```cpp
void CongestionEvent::apply(TrafficGraph& graph)
{
    Road* r = graph.get_edge(targetId_);
    if (r)
    {
        r->add_event_congestion(intensity_); // Increased traffic congestion
        r->update_travel_cost();             // Update travel cost
    }
}

void CongestionEvent::expire(TrafficGraph& graph)
{
    Road* r = graph.get_edge(targetId_);
    if (r)
    {
        r->add_event_congestion(-intensity_);  // clamped to >= 0 internally
        r->update_travel_cost();
    }
}
```

**Meaning:** `apply` looks up the target road by id (`graph.get_edge`
returns a raw `Road*`, `nullptr` if the id doesn't resolve), and if found,
adds `intensity_` to the road's congestion state, then recomputes the
road's routing cost. `expire` does the exact mirror-image: subtracts the
same `intensity_` back out, then recomputes cost again. The inline comment
notes the congestion accumulator is "clamped to >= 0 internally" inside
`Road` — meaning `CongestionEvent` doesn't have to worry about
double-subtracting pushing congestion negative; `Road` itself guards that
invariant.

**Why:** `apply`/`expire` being exact inverses of each other is the whole
contract this class exists to fulfill: an event that changes the graph
temporarily must be able to put it back exactly as it found it (modulo the
clamping `Road` does), or repeated events on the same road would drift the
graph's state over time. The `if (r)` null-check matters because `apply`
can run well after the event was validated to exist — see
`TrafficController::trigger_event`'s comment "the target has to exist, or
apply()/expire() would silently do nothing" — this is the exact silent-noop
behavior that comment is warning about; it's a deliberate defensive choice
(don't crash on a stale/deleted target) but it does mean a bug upstream
that lets a dangling `targetId_` through would fail *silently* here, not
loudly.

**Usage:** `AccidentEvent::apply`/`expire` and `RoadClosureEvent::apply`/
`expire` follow the identical shape (look up the road, mutate it, call
`update_travel_cost()`), differing only in *what* they mutate:
`AccidentEvent` calls `scale_capacity(0.3)` (capacity cut to 30%) plus
`add_event_congestion(0.7)`, undone in `expire` via `scale_capacity(1.0 /
0.3)` and `add_event_congestion(-0.7)`; `RoadClosureEvent` calls
`set_closed(true)`/`set_closed(false)`. `TrafficLightEvent::apply`/`expire`
is the one structurally different pair — it looks up an *intersection*, not
a road (`graph.intersections.find(targetId_)`), and only proceeds `if
(node && node->light_system)` — an intersection without a signal (a
stop-sign or uncontrolled junction) simply can't have this event applied,
another defensive `if` guarding a domain invariant rather than trusting the
caller. It scales `TrafficLight`'s green duration by `kBrokenGreenScale`
(`0.2`) on `apply` and divides by the same constant on `expire` to restore
it exactly (see `learn/03-traffic-lights.md` for `TrafficLight` itself).

```cpp
TrafficNotification CongestionEvent::startNotification() const
{
    return {Kind::Congestion, targetId_, "", description()};
}

TrafficNotification CongestionEvent::endNotification() const
{
    return {Kind::CongestionCleared, targetId_, "", "Congestion cleared on " + targetId_};
}
```

**Meaning:** Aggregate-initialization syntax — `{Kind::Congestion,
targetId_, "", description()}` constructs a `TrafficNotification` in field
order: `kind`, `roadId`, `intersectionId`, `description`. `startNotification`
uses the `Congestion` kind and calls `description()` (the shared "Heavy
congestion on road X" text); `endNotification` uses `CongestionCleared` and
writes its own inline string rather than reusing `description()`, since the
"cleared" message is different text, not just a different kind. Note
`intersectionId` is left `""` here — this is a road-scoped event, not an
intersection-scoped one. `TrafficLightEvent`'s versions do the mirror
image: they fill `intersectionId` with `targetId_` and leave `roadId` as
`""`.

**Why:** This is exactly the "one description of what an event means, not
two" idea called out in `TrafficController.hpp`'s header comment (section
3) — the event itself is the single source of truth for what notification
its start/end produces; `TrafficController` never constructs a
`TrafficNotification` by hand, it always asks the event for one. That
keeps the `roadId`-vs-`intersectionId` scoping rule (road events fill
`roadId`, light events fill `intersectionId`) co-located with the event
type that has to get it right, instead of being reimplemented (and
potentially gotten wrong) at every broadcast call site.

**Usage:** `TrafficController::trigger_event` calls
`event->startNotification()` right after `apply()` and passes the result
straight to `notifyAll`; `expire_events_targeting` and `update` call
`event->endNotification()` at the matching undo point. `Vehicle::onNotify`
(covered in `learn/05-vehicles.md`) is the consumer on the other end — it
switches on `.kind` and reads `.roadId` to decide whether to reroute.

```cpp
std::unique_ptr<TrafficEvent> makeEvent(const std::string& type,
                                        const std::string& id,
                                        const std::string& targetId,
                                        double durationSeconds) {
    if (type == "congestion") {
        return std::make_unique<CongestionEvent>(id, targetId, durationSeconds);
    }
    if (type == "accident") {
        return std::make_unique<AccidentEvent>(id, targetId, durationSeconds);
    }
    if (type == "closure") {
        return std::make_unique<RoadClosureEvent>(id, targetId, durationSeconds);
    }
    if (type == "light") {
        return std::make_unique<TrafficLightEvent>(id, targetId, durationSeconds);
    }
    return nullptr;
}
```

**Meaning:** A straight `if`-chain string dispatch (not a lookup table or a
`std::map<std::string, ...>` of constructors) mapping the four known type
strings to `std::make_unique<...>` calls for the matching concrete class.
Any unrecognized `type` string falls through every branch and returns
`nullptr`.

**Why:** `std::make_unique` is preferred over `std::unique_ptr<T>(new
T(...))` here for the usual reason — it avoids naming `T` twice and, more
importantly, avoids a narrow exception-safety gap where `new T(...)` could
succeed but the `unique_ptr` constructor wrapping it never runs if
something between them throws. Returning `nullptr` on an unknown type
string (rather than throwing) matches how the sole caller,
`TrafficController::trigger_event`, uses it: `if (!event) return "";  //
unknown type` — an unrecognized type is treated as an ordinary "couldn't
create this event" outcome, not an exceptional program error, consistent
with the rest of `trigger_event`'s validation (empty target, missing graph
entity, etc.) all returning `""` rather than throwing.

**Usage:** See `TrafficController::trigger_event`, section 3 below, and the
HTTP API mentioned in the factory's doc comment.

**Check yourself:**
- Why does `CongestionEvent::expire` subtract `intensity_` rather than resetting congestion to some fixed baseline value?
- `TrafficLightEvent::apply`/`expire` guard with `if (node && node->light_system)` — what real-world situation does that guard actually protect against?
- Why does `makeEvent` return `nullptr` for an unknown type instead of throwing an exception?

---

## 3. `cpp_backend/include/simulation/TrafficController.hpp`

```cpp
#pragma once
/*
TrafficController.hpp

- Owns the live traffic events and is the Subject every Vehicle attaches to.

- Splitting this out of TrafficSimulator keeps two concerns apart: the
simulator moves vehicles, the controller decides what happens to the
network. It is also the natural place for the Observer subject, because the
things worth broadcasting -- a closure, an accident, a signal fault -- are
exactly the events it owns.

- The notifications are produced by the events themselves
(TrafficEvent::startNotification / endNotification), so there is one
description of what an event means, not two.
*/

#include <memory>
#include <random>
#include <string>
#include <vector>

#include "events/TrafficEvent.hpp"
#include "utils/Observer.hpp"

class TrafficGraph;

class TrafficController : public Subject {
```

**Meaning:** The header comment states the class's role and justifies its
existence as a *separate* class from `TrafficSimulator`, rather than
folding event management into the simulator directly. `class TrafficGraph;`
is a forward declaration — `TrafficController` only ever holds a
`TrafficGraph*` (a pointer), never a full `TrafficGraph` value, so it
doesn't need the complete type here; the `.cpp` includes `core/Graph.hpp`
for the definition it actually needs to dereference the pointer. The class
declaration itself, `class TrafficController : public Subject`, is the
concrete answer to "where does the codebase plug into the generic
`Subject`/`IObserver` machinery from section 1" — this is the line that
makes `TrafficController` *be* a Subject, inheriting `attach`, `detach`,
and `notifyAll` from `Observer.hpp` rather than reimplementing them.

**Why:** This is worth pausing on directly, since it's the whole point of
the module: `TrafficController` is the **Subject** in this codebase's
Observer pattern. `Vehicle` (in `learn/05-vehicles.md`) is the
**Observer**. The **payload** is `TrafficNotification`
(`utils/Observer.hpp`, section 1). The **events** that trigger broadcasts
are the `TrafficEvent` hierarchy (section 2) — but note the events
themselves are neither Subject nor Observer; they're just data/behavior
objects that `TrafficController` drives and that happen to know how to
produce a `TrafficNotification` when asked. The public inheritance
(`public Subject`, not `private Subject`) is what makes `attach`/`detach`/
`notifyAll` visible and callable on a `TrafficController` from the outside
— `TrafficSimulator` calls `controller_.attach(v)` and
`controller_.detach(v)` directly (see Usage below), which requires those
methods to be public through the inheritance, not just usable internally.
As for *why* this Subject role sits on `TrafficController` rather than on
`TrafficSimulator` itself: the header comment gives the real reasoning —
separating "what moves vehicles" from "what changes the network" keeps
each class's responsibility narrow, and the Subject role naturally belongs
wherever the broadcastable events live, which is here.

**Usage:** `TrafficSimulator` owns one `TrafficController controller_` as a
member (`cpp_backend/include/simulation/TrafficSimulator.hpp:240`, "owns
the events, Subject for vehicles") and delegates to it rather than
reimplementing event or observer logic itself — see
[`learn/07-simulation-core.md`](07-simulation-core.md) for the tick loop
that calls `controller_.update(dt)` every frame.

```cpp
public:
    TrafficController() = default;

    void set_graph(TrafficGraph* graph);

    std::string trigger_event(const std::string& type, std::string target_id,
                              double duration);

    int expire_events_targeting(const std::string& target_id);

    void update(double dt);

    void clear();

    const std::vector<std::unique_ptr<TrafficEvent>>& events() const
    { 
        return events_; 
    }

    const std::vector<std::string>& log() const 
    { 
        return log_; 
    }

    void log_line(const std::string& line); 

    void set_random_events(bool on) 
    {
        random_events_ = on; 
    }

    bool random_events() const 
    { 
        return random_events_; 
    }
```

**Meaning:** `TrafficController() = default;` — no custom constructor
logic needed; every member has an in-class default (`graph_ = nullptr`,
empty vectors, `next_event_id_ = 1`, etc., visible in the private section
below). `set_graph` repoints the controller at a (possibly new) map.
`trigger_event` is the primary entry point for starting an event: it takes
a type string, a target id (by value, since the implementation may need to
overwrite an empty one — see section body below), and a duration, returning
the new event's id or `""` on failure. `expire_events_targeting` force-ends
every live event on a given target immediately (used before deleting a
road). `update(double dt)` is the per-tick driver: advances every event,
expires finished ones, and occasionally fires a random background event.
`clear()` drops every event *without* undoing it — the doc comment
("`Only for a full reset, where the graph is being thrown away anyway`")
explains why that's safe: if the whole graph is about to be discarded,
there's no `Road`/`Intersection` state left to bother restoring.
`events()` and `log()` are const accessors exposing read-only views for
introspection (the API/UI layer, per the doc comments — see
`learn/09-server-and-entrypoint.md`). `log_line` appends a line to an
internal ring-bounded log. `set_random_events`/`random_events` toggle and
query whether the controller injects events on its own.

**Why:** `trigger_event` returning a `std::string` id (not a `bool`, not
throwing) mirrors the "empty string means failure" convention already seen
in `makeEvent`/`Road` code throughout this module — a single sentinel value
(`""`) rather than an out-parameter or an exception for what's treated as
an ordinary, expected failure mode (bad target id, unknown type, etc.).
Returning `const std::vector<...>&` from `events()`/`log()` (references,
not copies) avoids copying potentially many `unique_ptr`s or log lines just
to let a caller iterate — but note this also means the returned reference
is only valid as long as `controller_` doesn't mutate `events_`/`log_`
underneath the caller, an implicit lifetime contract typical of accessor
methods returning containers by reference.

**Usage:** `set_graph` is called from `TrafficSimulator` whenever the
active map changes (`TrafficSimulator.cpp:75`) and on shutdown
(`TrafficSimulator.cpp:61`, with `nullptr`). `trigger_event` is called both
by `TrafficController::update`'s internal random-event logic and directly
by `TrafficSimulator::trigger_event` (`TrafficSimulator.cpp:442`), which is
itself presumably called from the HTTP API layer covered in
`learn/09-server-and-entrypoint.md`. `expire_events_targeting` is called by
`MapEditor` right before a road is deleted (see the header doc comment
about not leaving "the graph...holding the effects of an event whose
target has gone" — this connects directly to the dangling-pointer-style
hazard theme running through this whole module, just applied to event
targets instead of observer pointers).

```cpp
private:
    std::string random_road() const;
    std::string random_signalized_node() const;

    TrafficGraph* graph_ = nullptr;
    std::vector<std::unique_ptr<TrafficEvent>> events_;
    std::vector<std::string> log_;
    int next_event_id_ = 1;

    bool random_events_ = true;
    double random_event_timer_ = 20.0;
    mutable std::mt19937 rng_{20260729u};  // fixed seed: replays are reproducible
};
```

**Meaning:** Two private helpers pick a random road or a random
signal-equipped intersection (used when `trigger_event` is called with an
empty target). The state: `graph_` (non-owning, may be `nullptr`),
`events_` (the owned collection of live events — `unique_ptr` because
`TrafficController` is the sole owner and needs polymorphic storage of the
four concrete event types through the abstract base), `log_` (bounded
human-readable history), `next_event_id_` (monotonic counter for
generating ids like `"E1"`, `"E2"`, ...), `random_events_` (on/off switch),
`random_event_timer_` (countdown to the next random event), and `rng_` — a
`std::mt19937` Mersenne Twister engine, marked `mutable` (so it can be
advanced even from the `const` methods `random_road`/
`random_signalized_node`), seeded with the fixed literal `20260729u`
rather than a time-based seed, with the inline comment explaining exactly
why: "replays are reproducible."

**Why:** The fixed RNG seed is a deliberate, notable design choice worth
flagging on its own: most simulations seed their RNG from
`std::random_device` or the clock specifically to get different behavior
each run; this codebase does the opposite, on purpose, so that a
recorded/replayed simulation session produces byte-identical random events
every time it's re-run — valuable for debugging ("why did the simulation
gridlock at tick 4000") and for any deterministic-replay feature elsewhere
in the project. `mutable` on `rng_` is required specifically because
`random_road()`/`random_signalized_node()` are declared `const` (they don't
conceptually mutate the *controller's* externally-visible state — the graph
they read isn't changed) yet still need to advance the RNG's internal
state to produce different random picks call to call; without `mutable`,
those `const` methods couldn't touch `rng_` at all.

**Usage:** `random_road`/`random_signalized_node` are called from
`trigger_event` when `target_id` arrives empty (an unaimed event — e.g.
"put an accident somewhere"). The countdown/reseed pattern for
`random_event_timer_` lives in `update`'s implementation (below).

**Check yourself:**
- Why is `rng_` marked `mutable`, and what would fail to compile without it, given that `random_road()` is a `const` method?
- Trace the class hierarchy: which class is the Subject, which is the Observer, and which class produces the payload they exchange? Point to the exact line where `TrafficController` acquires its Subject role.
- Why does `clear()` skip calling `expire()` on every event, while `expire_events_targeting()` does not?

---

## `cpp_backend/src/simulation/TrafficController.cpp`

```cpp
#include "simulation/TrafficController.hpp"

#include <algorithm>

#include "core/Graph.hpp"
#include "core/Intersection.hpp"
#include "core/Road.hpp"

namespace {
    double default_duration(const std::string& type) {
        if (type == "closure") return 60.0;
        if (type == "accident") return 50.0;
        if (type == "light") return 40.0;
        return 45.0; //congestion event
    }

    constexpr std::size_t kMaxLog = 30;
}
```

**Meaning:** An anonymous (unnamed) `namespace` gives `default_duration`
and `kMaxLog` internal linkage — they're only visible inside this
translation unit, not exported to anything that links against
`traffic_core`. `default_duration` maps event type strings to
per-type default lifetimes in seconds if the caller didn't specify one.
`kMaxLog` (`30`) bounds how many lines `log_` keeps.

**Why:** The anonymous namespace is the modern C++ idiom for "this is a
private implementation detail of this one `.cpp` file" — equivalent in
effect to the old C convention of `static` free functions, but idiomatic
C++. Each event type gets a different default duration reflecting its
real-world severity/disruption: a closure (60s) outlasts an accident
(50s), which outlasts a light fault (40s); congestion (45s, the fallback
default) sits in between. These numbers are tuning knobs for how the
simulated city "feels," not derived from any formula.

```cpp
void TrafficController::set_graph(TrafficGraph* graph) {
    events_.clear();  //clear all event from old map
    graph_ = graph;
    next_event_id_ = 1;
    random_event_timer_ = 20.0;
}
```

**Meaning:** Drops every currently-tracked event (without calling
`expire()` on any of them — same reasoning as `clear()`), repoints
`graph_` at the new map, resets the id counter back to 1, and resets the
random-event countdown to its initial value.

**Why:** This runs whenever the active map changes (a fresh simulation, or
loading a different map file) — the old graph's `Road`/`Intersection`
objects are about to be replaced or destroyed entirely, so any lingering
event state that referred to them (`targetId_` pointing at ids that may no
longer exist in the new graph) would be meaningless, not just stale.
Resetting `next_event_id_` back to `1` means event ids like `"E1"` are
reused across separate simulation runs — acceptable because ids only need
to be unique *within* one graph's lifetime, not globally across the
process.

**Usage:** Called by `TrafficSimulator` (`TrafficSimulator.cpp:75`) every
time a map is loaded, and with `nullptr` on shutdown
(`TrafficSimulator.cpp:61`) — after which `trigger_event`/`update` become
no-ops (both check `if (!graph_) return`).

```cpp
std::string TrafficController::random_road() const {
    if (!graph_ || graph_->edges.empty()) return "";
    std::uniform_int_distribution<std::size_t> pick(0, graph_->edges.size() - 1);
    auto it = graph_->edges.begin();
    std::advance(it, pick(rng_));
    return it->first;
}
```

**Meaning:** Guards against no graph or an empty edge map. Builds a
uniform integer distribution over the valid index range of
`graph_->edges`, draws one index using `rng_`, then walks an iterator
forward that many steps (`std::advance`) from `edges.begin()` and returns
its key (`it->first` — `edges` is evidently a map keyed by road id, since
`.first` yields something returned as a road id string).

**Why:** `std::advance`-ing an iterator by a random offset is the standard
way to pick a uniformly random element out of an associative container
(like `std::map`/`std::unordered_map`) that doesn't support random-access
indexing (`operator[]` by position, unlike `std::vector`) — this is an
`O(n)` walk, acceptable here because it only runs on event-trigger, not
every tick.

**Usage:** Called from `trigger_event` when a non-`"light"` event arrives
with an empty target.

```cpp
std::string TrafficController::random_signalized_node() const {
    if (!graph_ || graph_->intersections.empty()) return "";
    std::uniform_int_distribution<std::size_t> pick(0, graph_->intersections.size() - 1);
    for (int attempt = 0; attempt < 64; ++attempt) {
        auto it = graph_->intersections.begin();
        std::advance(it, pick(rng_));
        if (it->second && it->second->is_signalized() && it->second->light_system) {
            return it->first;
        }
    }
    for (const auto& [id, node] : graph_->intersections) {
        if (node && node->is_signalized() && node->light_system) return id;
    }
    return "";  // nothing lit on this map: the event is skipped
}
```

**Meaning:** Same random-pick machinery as `random_road`, but with an added
constraint: the picked intersection must be signalized *and* actually have
a `light_system`. Since not every intersection has a light, this loops up
to 64 random attempts hoping to land on a qualifying one. If all 64 misses,
it falls back to a deterministic linear scan of every intersection,
returning the first one that qualifies — guaranteeing a correct answer (if
one exists) rather than giving up just because random sampling got unlucky.
If truly no intersection on the map has a light, it returns `""`.

**Why:** The 64-attempt-then-linear-scan structure is a pragmatic
trade: on a map where most intersections *are* signalized, random sampling
finds a hit almost immediately (cheap, `O(1)`-ish expected case); on a map
where signals are sparse, random sampling alone could in theory loop
forever (or need a very high retry cap) hoping to get lucky, so the
fallback linear scan (`O(n)`, but bounded and guaranteed-terminating)
covers the worst case without ever needing an unbounded retry loop. This
is a small but real defensive-engineering detail — someone thought about
what happens on a lights-sparse map and didn't let random chance alone
determine whether `trigger_event("light", "", 0)` can ever succeed.

**Usage:** Called from `trigger_event` specifically for `type == "light"`
with an empty target — traffic-light-fault events need a signalized
target, unlike the other three event types which can land on any road.

```cpp
std::string TrafficController::trigger_event(const std::string& type,
                                             std::string target_id,
                                             double duration) {
    if (!graph_) return "";

    if (target_id.empty()) {
        target_id = (type == "light") ? random_signalized_node() : random_road();
    }
    if (target_id.empty()) return "";

    if (type == "light") {
        auto it = graph_->intersections.find(target_id);
        if (it == graph_->intersections.end() || !it->second->light_system) return "";
    } else {
        if (!graph_->get_edge(target_id)) return "";
    }

    if (type == "closure") {
        const Road* road = graph_->get_edge(target_id);
        if (road && road->is_closed()) return "";
    }

    if (duration <= 0.0) duration = default_duration(type);

    const std::string id = "E" + std::to_string(next_event_id_++);
    std::unique_ptr<TrafficEvent> event = makeEvent(type, id, target_id, duration);
    if (!event) return "";  // unknown type

    event->apply(*graph_);
    notifyAll(event->startNotification());
    log_line(event->description());
    events_.push_back(std::move(event));
    return id;
}
```

**Meaning:** Walks through validation before ever constructing an event:
(1) bail if there's no graph at all; (2) if no target was given, pick a
random one appropriate to the type; (3) bail if even that couldn't find a
target; (4) re-verify the (now-resolved) target actually exists in the
graph and, for `"light"`, actually has a `light_system` — this re-check
matters even for caller-supplied (non-random) target ids, since a caller
(e.g. the HTTP API) could pass any string; (5) specifically for
`"closure"`, refuse if the road is *already* closed, with the inline
comment explaining exactly why: a second closure event's `expire()` would
reopen the road while the first closure is still supposed to be active;
(6) fall back to `default_duration(type)` if the caller passed a
non-positive duration; (7) mint a fresh id, hand everything to `makeEvent`
(section 2), bail if the type string was unrecognized. Once an event object
exists: **apply it to the graph first** (`event->apply(*graph_)`), **then**
broadcast (`notifyAll(event->startNotification())`), **then** log a
human-readable line, **then** finally store the event by moving the
`unique_ptr` into `events_`, and return its id.

**Why:** This is the moment the whole module's machinery converges: an
event only gets a chance to notify observers *after* it has already been
proven mechanically valid and has already mutated the graph — the ordering
`apply()` → `notifyAll()` → `log_line()` → store is significant. If
`notifyAll` ran *before* `apply`, a `Vehicle::onNotify` handler could react
to "this road just closed" while the graph itself still shows the road
open — a race between announcement and reality, even in a single-threaded
tick loop, simply from getting the statement order wrong. Doing `apply`
first guarantees any observer reacting synchronously to the notification
sees a graph that's already consistent with what it's being told. The
duplicate-closure guard is a direct instance of the same "keep apply/expire
as true inverses" theme from section 2 — allowing two overlapping closures
on one road would break that invariant the moment either one expired.

**Usage:** This is the method every event-creation path funnels through —
called by the random-event logic inside `update` (below) with an empty
target_id, and by `TrafficSimulator::trigger_event`
(`TrafficSimulator.cpp:442`) forwarding a client's HTTP request.

```cpp
int TrafficController::expire_events_targeting(const std::string& target_id) {
    if (!graph_) return 0;

    int expired = 0;
    for (auto& event : events_) {
        if (event->target() != target_id || event->expired()) continue;
        event->expire(*graph_);
        notifyAll(event->endNotification());
        log_line(event->endNotification().description);
        ++expired;
    }
    if (expired > 0) {
        events_.erase(std::remove_if(events_.begin(), events_.end(),
                                     [&](const std::unique_ptr<TrafficEvent>& e) {
                                         return e->target() == target_id;
                                     }),
                      events_.end());
    }
    return expired;
}
```

**Meaning:** Iterates every live event; for each one matching `target_id`
that *isn't already expired*, calls its `expire()` (undo the graph
mutation), broadcasts the end notification, logs it, and counts it.
`event->endNotification()` is evaluated **twice** here — once passed to
`notifyAll`, once to fetch `.description` for the log — rather than storing
the result once in a local variable. After the loop, if anything was
expired, a second pass with erase-remove drops every event on that target
from `events_` (not just the ones expired in the loop — but since the loop
condition already skips already-`expired()` events and every event *not*
skipped just had `.expire()` called on it, this removal step is really
just cleaning out everything targeting this id, expired-now or
already-expired-earlier-but-not-yet-erased).

**Why:** This mirrors the exact same erase-remove idiom from
`Subject::detach` (section 1) — same C++ pattern, applied to a different
container, for the same reason (mutate a vector by dropping matching
entries without leaving unspecified tail elements around). Calling
`endNotification()` twice instead of caching it once is a minor
inefficiency (constructs two temporary `TrafficNotification`s, including
two string concatenations for the description text, instead of one) — not
a correctness bug, since both calls are `const` and pure, just a small
missed opportunity worth noticing as exactly the kind of "genuinely a
minor rough edge" the house style asks to be called out rather than
glossed over.

**Usage:** Called by `MapEditor` right before a road is physically deleted
from the graph, per the header's doc comment — so that no leftover
`AccidentEvent`/`CongestionEvent`/etc. is left holding a reference to a
road id that's about to stop existing.

```cpp
void TrafficController::update(double dt) {
    if (!graph_) return;

    for (auto& event : events_) {
        const bool was_expired = event->expired();
        event->advance(dt);
        if (!was_expired && event->expired()) {
            event->expire(*graph_);
            notifyAll(event->endNotification());
            log_line(event->endNotification().description);
        }
    }
    events_.erase(std::remove_if(events_.begin(), events_.end(),
                                 [](const std::unique_ptr<TrafficEvent>& e) {
                                     return e->expired();
                                 }),
                  events_.end());

    if (!random_events_) return;
    random_event_timer_ -= dt;
    if (random_event_timer_ > 0.0) return;

    std::uniform_real_distribution<double> next(18.0, 45.0);
    random_event_timer_ = next(rng_);
    static const char* kTypes[] = {"congestion", "accident", "closure", "light",
                                   "congestion"};
    std::uniform_int_distribution<int> pick(0, 4);
    trigger_event(kTypes[pick(rng_)], "", 0.0);
}
```

**Meaning:** The per-tick driver, called once per simulation frame. First
loop: for every live event, snapshot whether it was *already* expired
before this tick (`was_expired`), advance its clock by `dt`, and — only if
it *transitioned* from not-expired to expired this very tick (`!was_expired
&& event->expired()`) — call `expire()`/broadcast/log exactly once. That
transition check is what stops an event from being expired repeatedly
every tick after it first crosses its duration; without it, an event still
sitting in `events_` after expiring (e.g. before the cleanup pass below
runs) would re-trigger `expire()` and a fresh broadcast every single frame.
After the loop, a second erase-remove pass physically drops every expired
event from `events_`. Then, only if `random_events_` is on: count down
`random_event_timer_` by `dt`; if it hasn't reached zero yet, return early
(no random event this tick); otherwise draw a new random interval (18 to 45
seconds) for the *next* random event, then pick one of five weighted type
slots — `"congestion"` appears twice in the five-element array, doubling
its odds relative to `"accident"`/`"closure"`/`"light"`, with the inline
comment explaining why: "it is the mildest event, so it should be the
common one" — and call `trigger_event` with that type, an empty target
(so `random_road`/`random_signalized_node` picks one), and `0.0` duration
(so `trigger_event` falls back to `default_duration`).

**Why:** The `was_expired`/transition-check pattern is the tick-loop
analogue of the duplicate-closure guard in `trigger_event` — both exist to
prevent the same class of bug, an action being repeated when it should
happen exactly once. Weighting the random type array by literal repetition
(`{"congestion", "accident", "closure", "light", "congestion"}`, picked via
`uniform_int_distribution<int> pick(0, 4)`) rather than, say, a proper
weighted-discrete-distribution (`std::discrete_distribution`) is a simple,
slightly informal way to get non-uniform odds — easy to read, easy to
rebalance by adding/removing array entries, at the cost of odds being
implicit in array contents rather than an explicit weight table. This
`update` method is also where the "keep the city alive" feel described in
the header comment actually comes from — without the random-event branch,
nothing would ever happen in a simulation unless a client explicitly
triggered something over the API.

**Usage:** `TrafficSimulator`'s tick function calls
`controller_.update(dt)` once per simulated frame
(`TrafficSimulator.cpp:172`) — see
[`learn/07-simulation-core.md`](07-simulation-core.md) for the surrounding
tick loop and how `dt` is computed. This is the method that ultimately
drives every `notifyAll` call in ordinary operation (the random-event
path), with `trigger_event`'s API-triggered path as the other source.

**Check yourself:**
- In `trigger_event`, why is the graph re-validated (`graph_->intersections.find`/`get_edge`) even when `target_id` was just supplied by `random_road`/`random_signalized_node`, which by construction only return valid ids?
- Walk through what would go wrong in `update` if the `was_expired` transition check were removed and `expire()`/`notifyAll` were called unconditionally whenever `event->expired()` is true.
- Why does the random-event type array list `"congestion"` twice instead of using `std::discrete_distribution` with explicit weights?
- `expire_events_targeting` and `update`'s cleanup pass both use the erase-remove idiom on `events_` — where else in this module does the exact same idiom appear, and why does it keep coming up?

---

## Tying the pattern together

To make the Observer Pattern's three roles explicit, since it's easy to
lose track across three files:

- **Subject** — `TrafficController` (`class TrafficController : public
  Subject`, `TrafficController.hpp`). It inherits `attach`/`detach`/
  `notifyAll` from `Subject` (`utils/Observer.hpp`) rather than
  reimplementing them.
- **Observer** — `Vehicle` (`class Vehicle : public IObserver`,
  `vehicles/Vehicle.hpp`, covered in
  [`learn/05-vehicles.md`](05-vehicles.md)). It implements the one pure
  virtual, `onNotify`, by switching on `TrafficNotification::Kind` and
  setting `needs_reroute_` when an event affects a road still ahead on its
  route.
- **Payload** — `TrafficNotification` (`utils/Observer.hpp`), produced by
  whichever `TrafficEvent` subclass (`events/TrafficEvent.hpp`) just
  started or ended.
- **The wiring**, concretely, in `TrafficSimulator.cpp`: `attach` happens
  once at `controller_.attach(v)` in `spawn_vehicle`
  (`TrafficSimulator.cpp:431`); `detach` happens on every vehicle-removal
  path; `notifyAll` fires from inside `TrafficController::trigger_event`,
  `expire_events_targeting`, and `update` — never called directly by
  `TrafficSimulator` itself, since `TrafficController` owns that
  responsibility end to end.

This whole chain is what lets `learn/07-simulation-core.md`'s tick loop
stay simple: one call, `controller_.update(dt)`, and every live event
advances, expires, and (if relevant) tells every vehicle that cares —
without the tick loop itself needing to know a single thing about roads,
accidents, or vehicle rerouting logic.
