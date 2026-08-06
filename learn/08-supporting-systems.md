# Module 08 — Supporting Systems

**Files covered:**
- [`cpp_backend/include/utils/Json.hpp`](../cpp_backend/include/utils/Json.hpp) / [`cpp_backend/src/utils/Json.cpp`](../cpp_backend/src/utils/Json.cpp)
- [`cpp_backend/include/utils/MapLoader.hpp`](../cpp_backend/include/utils/MapLoader.hpp) / [`cpp_backend/src/utils/MapLoader.cpp`](../cpp_backend/src/utils/MapLoader.cpp)
- [`cpp_backend/include/simulation/MapEditor.hpp`](../cpp_backend/include/simulation/MapEditor.hpp) / [`cpp_backend/src/simulation/MapEditor.cpp`](../cpp_backend/src/simulation/MapEditor.cpp)
- [`cpp_backend/include/simulation/HistoryStore.hpp`](../cpp_backend/include/simulation/HistoryStore.hpp) / [`cpp_backend/src/simulation/HistoryStore.cpp`](../cpp_backend/src/simulation/HistoryStore.cpp)
- [`cpp_backend/include/utils/StatisticsManager.hpp`](../cpp_backend/include/utils/StatisticsManager.hpp) / [`cpp_backend/src/utils/StatisticsManager.cpp`](../cpp_backend/src/utils/StatisticsManager.cpp)

Everything in this module sits around the tick loop rather than in it. Two
JSON encoders (one from-scratch, one vendored), a GeoJSON/OSM ingestion
pipeline that builds the `TrafficGraph` covered in
[`learn/02-core-graph-model.md`](02-core-graph-model.md), a live map-editing
component, a disk-backed playback recorder, and a metrics aggregator. None of
these mutate vehicle physics — they exist so the simulator has data to load,
a way to be edited at runtime, a way to be scrubbed backward in time, and a
way to report on itself. All four consumers are wired directly into
`TrafficSimulator` (see [`learn/07-simulation-core.md`](07-simulation-core.md)):
`map_editor_`, `history_`, and `stats_` are member variables of the Singleton
itself, and `MapLoader` is called as a set of static functions from
`TrafficSimulator::load()` and its map-editing API.

---

## `cpp_backend/include/utils/Json.hpp` and `cpp_backend/src/utils/Json.cpp`

### Lines 1–17 (header): a hand-rolled writer, not a parser

```cpp
/*
    - This is a simple helper for writing JSON by hand.
    - Note: We deliberately use this hand-rolled JSON helper here instead of
      the vendored nlohmann/json library. This is to avoid pulling a heavy
      header into a hot path, keeping serialization fast and lightweight.
*/
#pragma once

#include <string>

namespace json {

std::string escape(const std::string& in);
std::string str(const std::string& s);
std::string num(double v, int precision);

}
```

**Meaning:** the whole public surface is three free functions in a `json`
namespace: `escape` (make a string JSON-safe), `str` (quote and escape a
string), `num` (format a `double` at a fixed precision). There is no `Value`
type, no parsing, no object/array builder — this is a *writer*, not a JSON
library.

**This is the answer the task brief asked for explicitly: `utils/Json.hpp`
is not a thin wrapper around `nlohmann::json`, and it is not a from-scratch
*parser* either — it's a from-scratch *serialization* helper, three
functions wide.** Reading the vendored library confirms the split: the
project keeps a full third-party JSON library at
`cpp_backend/include/third_party/nlohmann/json.hpp` for *parsing* untrusted
input (`MapLoader.cpp` uses `nlohmann::json::parse` on GeoJSON below), but
for *writing* the high-frequency state snapshot that goes out over the
WebSocket every tick, it deliberately avoids that library and hand-rolls
string concatenation instead.

**Why:** the comment states the rationale directly — `nlohmann::json` is a
large single header that builds a DOM-like `json` object graph with
allocation and type-erasure overhead per value. For a one-shot GeoJSON parse
at startup, that overhead is irrelevant. For the per-tick state broadcast
(every vehicle, every light, every congested road, sent to every connected
client on every frame), it is not — building a `nlohmann::json` tree just to
immediately `.dump()` it and throw it away would mean allocating a tree the
serializer walks once and discards, every single tick. Direct
`std::ostringstream`/string concatenation (as seen in
`StatisticsManager::toJson` and `HistoryStore::to_state_json` below) skips
that tree entirely. This is a deliberate asymmetry: correctness-critical,
infrequent work (parsing a map file) gets the safe, well-tested library;
performance-critical, frequent work (writing a frame) gets a handful of
inlined helpers.

**Usage:** `json::escape`/`json::str`/`json::num` are used anywhere
hand-written JSON output needs a correctly escaped string or a
precision-controlled number — `StatisticsManager::toJson` (below) is the
clearest example, and `TrafficSimulator::state_json()` (see
[`learn/07-simulation-core.md`](07-simulation-core.md)) is the actual hot
path this was built for.

---

### `Json.cpp` lines 8–30: `escape`

```cpp
string json::escape(const string& in)
{
    std::string out;
    out.reserve(in.size() + 8);
    for (char c : in) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned char>(c));
                    out += buf;
                } else {
                    out += c;
                }
        }
    }
    return out;
}
```

**Meaning:** walks the input string one `char` at a time. `out.reserve(in.size() + 8)`
pre-sizes the output buffer to roughly the input's size (plus a small pad
for escape sequences) so the loop doesn't repeatedly reallocate as it
appends. The `switch` handles the JSON-mandated escapes for quote,
backslash, newline, carriage return, and tab explicitly by name. The
`default` branch catches every other control character (any byte below
`0x20`, i.e. below the printable ASCII range) and emits it as a `\uXXXX`
escape via `snprintf` into a small stack buffer; anything else (ordinary
printable text, and any byte `>= 0x20` including raw UTF-8 continuation
bytes) is copied through unchanged.

**Why:** JSON strings cannot contain a literal `"`, `\`, or raw control
character — they'd either terminate the string early or produce invalid
JSON a client-side `JSON.parse` would reject. `static_cast<unsigned char>(c)`
matters here: `char` is signedness-unspecified in C++, so comparing a
`char` with a high bit set (e.g. a UTF-8 continuation byte, which is
negative as a signed `char`) directly against `0x20` would misbehave on
platforms where `char` is signed — the cast forces the comparison into the
0–255 range the check actually intends. Passing UTF-8 bytes `>= 0x20`
through unescaped (rather than also encoding them) is valid: JSON strings
are UTF-8 by default, so a raw UTF-8 sequence is already legal JSON content
and doesn't need per-byte escaping.

**Usage:** every string value written into the tick's JSON output — vehicle
IDs, road IDs, event descriptions — passes through here (directly or via
`json::str`) before hitting the wire. A vehicle whose ID or a building name
contained a `"` would corrupt the whole payload without this.

---

### `Json.cpp` lines 32–35 and 37–43: `str` and `num`

```cpp
string json::str(const std::string& s)
{
    return "\"" + escape(s) + "\"";
}

string json::num(double v, int precision)
{
    if (!std::isfinite(v)) return "null";
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.*f", precision, v);
    return std::string(buf);
}
```

**Meaning:** `str` wraps `escape`'s output in a literal pair of double
quotes — the complete, ready-to-embed JSON string token. `num` first checks
`std::isfinite(v)`: JSON has no representation for `NaN` or `±Infinity`, so
those get mapped to the JSON literal `null` rather than emitting something a
parser would reject or silently mis-load. Otherwise it formats `v` with
`snprintf`'s `%.*f` — the `*` takes the precision as a runtime argument
(the `precision` parameter) rather than being baked into the format string,
so callers control decimal places per call site (compare
`json::num(s.avgCongestion, 3)` vs `json::num(s.simTime, 1)` in
`StatisticsManager::toJson`).

**Why:** `%.*f` (fixed-point, not `%g`/scientific) guarantees output that's
always valid JSON number syntax and never surprises a client parser with
`1e-05`-style notation. Checking `isfinite` first is a defensive measure
against a corrupted/degenerate simulation state (e.g. a division by zero
somewhere upstream producing `NaN` congestion) — without it, a single bad
value would emit literal `nan` or `inf` text into the JSON stream, which
`JSON.parse` on the frontend would throw on and take down the *entire*
state update for every client, not just the one bad field.

**Usage:** `json::num` is the single choke point for every floating-point
value serialized by hand in this codebase (`StatisticsManager::toJson`,
`TrafficSimulator::state_json()`, see
[`learn/07-simulation-core.md`](07-simulation-core.md)). `HistoryStore::to_state_json`
notably does *not* use it — it streams doubles through `ostringstream`
directly, which is a difference worth noticing (see that section below).

**Check yourself:**
- Why does this project maintain two separate JSON code paths (a vendored
  parser and a hand-rolled writer) instead of using `nlohmann::json` for
  both directions?
- What JSON payload would `json::escape` fail to make safe, and why doesn't
  that failure mode matter here? (Hint: think about what's *not* a control
  character but is still meaningful to a JSON parser, and whether this
  codebase ever calls `escape` on such input.)
- Why does `json::num` map non-finite doubles to `"null"` instead of, say,
  `0`?

---

## `cpp_backend/include/utils/MapLoader.hpp` and `cpp_backend/src/utils/MapLoader.cpp`

This is the largest file in the module — over 1100 lines — because it does
two structurally different jobs behind one static-method API: loading this
project's *native* GeoJSON flavor (a format this project invented, described
fully in the class comment), and deriving a full road graph from a *raw
OpenStreetMap/Overpass export*, which was never designed to be a traffic
simulator's input format and has to be reverse-engineered into one. Both
paths build a [`TrafficGraph`](02-core-graph-model.md) — this is the
concrete "map data feeds Graph construction" link the syllabus predicted.

### `MapLoader.hpp` lines 1–28: the error type and the format contract

```cpp
class MapLoadError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

/**
 * Load a map from a SINGLE GeoJSON file.
 *
 * Format: a GeoJSON FeatureCollection where every feature has a "uts:kind"
 * property saying what it is (coordinates are already in world meters):
 *
 *   - "node"     Point       {id, name, zone, [signal, sgroups, sgreen, samber, sred]}
 *   - "road"     LineString  {id, from, to, [length, speed_kmh, capacity, lanes, oneway]}
 *   - "zone"     Polygon     {name, kind}          - display-only background region
 *   - "building" Polygon     {name, kind, levels, [node]}  - building with a footprint;
 *                            optional "node" anchors the building to a node as a destination
 *
 * A two-way street is stored as ONE feature (oneway=false) and expands into
 * two Road objects: "id" and "id_r" (the reverse direction).
 */
```

**Meaning:** `MapLoadError` is a tiny subclass of `std::runtime_error` that
adds no members of its own — `using std::runtime_error::runtime_error;` is a
using-declaration that pulls the base class's constructors into the derived
class, so `MapLoadError` can be constructed exactly like `runtime_error`
(from a `std::string` message) without writing that constructor by hand.
The class comment above `MapLoader` documents the "native" GeoJSON schema:
every feature carries a `uts:kind` property (`uts` = this project's own
namespace prefix, "urban traffic simulator") that says whether it's a node,
road, zone, or building, and each kind's expected property bag is spelled
out.

**Why:** a dedicated exception type (rather than throwing raw
`std::runtime_error` or returning an error code) lets callers distinguish
"this specific file/string failed to load as a map" from any other
`runtime_error` that might be flying around, via `catch (const MapLoadError&)`.
The `uts:kind` tag is what lets `load_from_geojson_string` (below) tell a
*native* map apart from a *raw OSM* export using nothing but the feature
data itself — no separate flag, no file extension convention, just "does
any feature carry this specific tag."

**Usage:** every failure path throughout this file (missing property,
malformed coordinates, duplicate ID, unreadable file) throws `MapLoadError`.
`TrafficSimulator::load()` and the map-import endpoint (see
[`learn/07-simulation-core.md`](07-simulation-core.md) and
[`learn/09-server-and-entrypoint.md`](09-server-and-entrypoint.md), where it
exists) catch it to report a clean error back to the caller instead of
crashing the process on a bad map file.

**Check yourself:**
- What does `using std::runtime_error::runtime_error;` actually save you
  from writing, and what would `MapLoadError` look like without it?
- Why key the native/OSM decision off feature content (`uts:kind`) rather
  than, say, a `--format=osm` command-line flag?

---

### `MapLoader.hpp` lines 29–60: the public API surface

```cpp
class MapLoader {
public:
    static void load_from_geojson(TrafficGraph& graph, const std::string& path);
    static void load_from_geojson_string(TrafficGraph& graph, const std::string& text,
                                         const std::string& source_name = "<string>");
    static std::string validate(const TrafficGraph& graph);
    static void write_map_json(const TrafficGraph& graph, const std::string& path);
    static void write_geojson(const TrafficGraph& graph, const std::string& path);
    static std::string map_json_string(const TrafficGraph& graph);
};
```

**Meaning:** six `static` methods, no instance state, no constructor — the
class is purely a namespace for related functions (a common C++ idiom when
there's no state to own; a free-function namespace would work identically,
but grouping under a class name reads as "this is a cohesive subsystem").
`load_from_geojson` reads a file path; `load_from_geojson_string` takes the
JSON text directly (the comment says "used by unit tests," meaning tests can
exercise the parser against inline strings without touching the filesystem).
`validate` runs a post-load sanity check. `write_map_json`/`map_json_string`
serialize the graph into the *display* format the frontend map renderer
consumes; `write_geojson` serializes it back into the *native* loadable
format, so a graph edited at runtime through `MapEditor` (below) can be
saved and reloaded losslessly.

**Why:** separating "load a file" from "load a string" is what makes the
loader unit-testable — tests can construct malformed or edge-case GeoJSON
inline and assert `MapLoadError` is thrown with a specific message, without
maintaining a directory of fixture files. Separating "the format for saving/
reloading" (`write_geojson`) from "the format for the frontend to draw"
(`write_map_json`/`map_json_string`) matters because they're genuinely
different shapes: the native GeoJSON needs enough fidelity to reconstruct
the exact graph (signal timing, roundabout flags, lane counts), while the
display JSON is a flattened, frontend-friendly shape that drops information
the renderer doesn't need (e.g. it emits each two-way street once, not as
two directional `Road` objects).

**Usage:** `TrafficSimulator::load()` calls `load_from_geojson` directly
(twice — once to validate into a scratch graph, once into the live graph,
per the grep below). `map_json_string` backs the `GET /api/map` HTTP
endpoint. `write_geojson` backs "save this edited city," and
`load_from_geojson_string` backs "import this uploaded map," both reachable
through the live map-editing forwarders in `TrafficSimulator` (see
[`learn/07-simulation-core.md`](07-simulation-core.md)).

**Check yourself:**
- Why does `MapLoader` have no instance state and only `static` methods,
  rather than being constructed once and holding, say, a reference to the
  graph it loads into?
- Trace why `write_geojson` and `write_map_json`/`map_json_string` can't be
  the same function — what information does one need that the other must
  drop?

---

### `MapLoader.cpp` lines 1–26: includes and the `nlohmann::json` alias

```cpp
#include "utils/MapLoader.hpp"

#include "core/Building.hpp"
#include "core/Graph.hpp"
#include "core/Intersection.hpp"
#include "core/Road.hpp"
#include "core/TrafficLight.hpp"
#include "core/Zone.hpp"
#include "third_party/nlohmann/json.hpp"
...
namespace {
using nlohmann::json;
```

**Meaning:** the loader pulls in every core graph type it constructs
(`Building`, `Intersection`, `Road`, `TrafficLight`, `Zone` — see
[`learn/02-core-graph-model.md`](02-core-graph-model.md) and
[`learn/03-traffic-lights.md`](03-traffic-lights.md)) plus the vendored
`nlohmann/json.hpp`. Everything below this point lives in an unnamed
namespace (`namespace { ... }`), meaning internal linkage — none of these
helper functions or types are visible outside this translation unit.
`using nlohmann::json;` brings the library's `json` type into scope
unqualified, so the rest of the file can write `json` instead of
`nlohmann::json` everywhere.

**Why:** this is the concrete evidence for the split documented in the
`Json.hpp` section above: *this* file, which parses arbitrary
(possibly-adversarial, possibly-malformed) file input exactly once at load
time, uses the full-featured vendored parser rather than hand-rolling one.
The unnamed namespace is a deliberate encapsulation choice — `str_prop`,
`num_prop`, `read_point`, `load_node`, `load_road`, and the entire OSM
ingestion pipeline below are implementation details of `MapLoader`'s public
methods; nothing outside this `.cpp` file should ever call them directly,
and the unnamed namespace makes that a compiler-enforced fact rather than a
convention.

**Usage:** every subsequent helper in this file is only reachable from the
six `MapLoader::` static methods defined at the bottom.

---

### `MapLoader.cpp` lines 28–68: the safe property readers

```cpp
std::string str_prop(const json& props, const char* key, const std::string& fallback = "") {
    auto it = props.find(key);
    if (it == props.end()) return fallback;
    if (it->is_string()) return it->get<std::string>();
    if (it->is_number_integer()) return std::to_string(it->get<long long>());
    return fallback;
}

double num_prop(const json& props, const char* key, double fallback) {
    auto it = props.find(key);
    if (it == props.end()) return fallback;
    if (it->is_number()) return it->get<double>();
    if (it->is_string()) {
        try { return std::stod(it->get<std::string>()); }
        catch (...) { return fallback; }
    }
    return fallback;
}

bool bool_prop(const json& props, const char* key, bool fallback) {
    auto it = props.find(key);
    if (it == props.end() || !it->is_boolean()) return fallback;
    return it->get<bool>();
}

bool read_point(const json& pt, double& x, double& y) {
    if (!pt.is_array() || pt.size() < 2 || !pt[0].is_number() || !pt[1].is_number()) {
        return false;
    }
    x = pt[0].get<double>();
    y = pt[1].get<double>();
    return true;
}
```

**Meaning:** four small readers, each taking a `json` object (`props`, the
GeoJSON feature's free-form `"properties"` bag), a key, and a fallback.
`str_prop` reads a string property but also accepts a JSON integer and
stringifies it (`id: 7` in a hand-edited file becomes `"7"`). `num_prop`
reads a numeric property but also accepts a numeric *string* (`"50"`),
falling back on a `std::stod` parse failure caught by a catch-all
`catch (...)`. `bool_prop` is strict — no coercion, just "is this key
present and actually a JSON boolean." `read_point` extracts an `[x, y]`
pair from a GeoJSON coordinate array, returning `false` (rather than
throwing) if the shape is wrong.

**Why:** GeoJSON as an interchange format is commonly hand-edited or
produced by tools with loose typing conventions, so being lenient about
`"7"` vs `7` for an ID, or `"50"` vs `50` for a speed, avoids rejecting
files that are semantically fine but not byte-perfect. `read_point`
returning `bool` instead of throwing is a deliberate "let the caller decide
how severe this is" design — some callers (`load_node`) treat a malformed
point as a hard `MapLoadError`, while others (`load_zone`, `load_building`)
just `continue`/skip the bad point and keep whatever *was* parseable,
because a zone or building's exact polygon shape is display-only and
degrading gracefully is preferable to discarding an entire building over
one bad coordinate.

**Usage:** every `load_*` function below reads its GeoJSON properties
exclusively through these four functions — never `props["key"]` directly —
which is what makes a missing/malformed optional property throw a `default`
instead of an unhandled `nlohmann::json` exception (which is a different,
less informative failure than `MapLoadError`).

**Check yourself:**
- Why does `str_prop` coerce integers to strings but `bool_prop` coerces
  nothing?
- What's the practical difference in this codebase between a function that
  throws `MapLoadError` on bad input (like `load_node`, below) and one that
  silently skips it (like `read_point` failures inside `load_zone`)?

---

### `MapLoader.cpp` lines 72–100: `load_node`

```cpp
void load_node(TrafficGraph& graph, const json& props, const json& geom,
               const std::string& source) {
    if (geom.value("type", "") != "Point") {
        throw MapLoadError("uts:kind=node feature has no Point geometry in " + source);
    }
    const std::string id = str_prop(props, "id");
    if (id.empty()) {
        throw MapLoadError("node feature is missing property 'id' in " + source);
    }
    if (graph.intersections.count(id)) {
        throw MapLoadError("duplicate node id '" + id + "' in " + source);
    }
    double x = 0.0, y = 0.0;
    if (!read_point(geom["coordinates"], x, y)) {
        throw MapLoadError("node '" + id + "' has malformed coordinates in " + source);
    }

    auto node = std::make_unique<Intersection>(id, x, y);
    node->display_name = str_prop(props, "name", id);
    node->zone = str_prop(props, "zone");
    node->is_roundabout = (node->zone == "roundabout");
    if (bool_prop(props, "signal", false)) {
        node->light_system = std::make_unique<TrafficLight>(
            num_prop(props, "sgreen", 25.0), num_prop(props, "samber", 3.0),
            num_prop(props, "sred", 2.0),
            static_cast<int>(num_prop(props, "sgroups", 2)));
    }
    graph.intersections[id] = std::move(node);
}
```

**Meaning:** four validation checks in sequence — geometry type, presence
of an `id`, uniqueness of that `id`, well-formed coordinates — each an
early-exit `throw`. Once validation passes, it constructs an `Intersection`
(see [`learn/02-core-graph-model.md`](02-core-graph-model.md)) via
`std::make_unique`, sets its display name (defaulting to the id itself if
no `"name"` property exists), its `zone`, and derives `is_roundabout`
straight from whether `zone == "roundabout"` — there's no separate
roundabout flag in the native format, the zone string *is* the signal.
If the `"signal"` property is truthy, it attaches a `TrafficLight` (see
[`learn/03-traffic-lights.md`](03-traffic-lights.md)) built from optional
`sgreen`/`samber`/`sred`/`sgroups` properties, each falling back to a
default if absent. Finally the fully-built node is moved into
`graph.intersections`, keyed by its own id.

**Why:** validating in this specific order (geometry, then id presence,
then id uniqueness, then coordinates) front-loads the checks that are
cheapest and most likely to catch a structurally wrong feature before doing
any real work. Throwing rather than skipping a bad node is the right choice
here (unlike zones/buildings, above) because a road later in the file will
reference this node by id — silently dropping a malformed node would turn
into a much more confusing "road references an unknown node" error far from
the actual mistake. `std::make_unique<Intersection>` then `std::move`
matches this project's consistent ownership convention: the graph owns its
nodes via `unique_ptr`, and everything else (roads, the arbiter) holds raw
non-owning pointers into it — the same pattern documented in
[`learn/02-core-graph-model.md`](02-core-graph-model.md).

**Usage:** called once per `uts:kind=node` feature, in "pass 1" of
`load_from_geojson_string`'s native-format handling (below), specifically
*before* roads are loaded, because `load_road` needs every referenced node
to already exist in `graph.intersections`.

---

### `MapLoader.cpp` lines 102–145: `load_road` and the two-way expansion

```cpp
void load_road(TrafficGraph& graph, const json& props, const std::string& source) {
    const std::string id = str_prop(props, "id");
    const std::string from = str_prop(props, "from");
    const std::string to = str_prop(props, "to");
    if (id.empty() || from.empty() || to.empty()) {
        throw MapLoadError("road feature is missing id/from/to in " + source);
    }
    auto from_it = graph.intersections.find(from);
    auto to_it = graph.intersections.find(to);
    if (from_it == graph.intersections.end() || to_it == graph.intersections.end()) {
        throw MapLoadError(...);
    }
    const bool oneway = bool_prop(props, "oneway", false);
    if (graph.edges.count(id) || (!oneway && graph.edges.count(id + "_r"))) {
        throw MapLoadError("duplicate road id '" + id + "' in " + source);
    }
    ...
    const bool circulating =
        bool_prop(props, "roundabout", false) ||
        (from_it->second->is_roundabout && to_it->second->is_roundabout);

    Intersection* from_node = from_it->second.get();
    Intersection* to_node = to_it->second.get();
    graph.edges[id] = std::make_unique<Road>(id, from_node, to_node, lanes, speed,
                                             capacity, oneway, length);
    graph.edges[id]->is_internal = circulating;
    if (!oneway) {
        graph.edges[id + "_r"] = std::make_unique<Road>(
            id + "_r", to_node, from_node, lanes, speed, capacity, false, length);
        graph.edges[id + "_r"]->is_internal = circulating;
    }
}
```

**Meaning:** validates `id`/`from`/`to` are present, that both endpoints
already exist as nodes (this is the "pass 2 needs pass 1 done first"
dependency mentioned above), and that the id isn't already taken — checking
both `id` and, for a two-way street, `id + "_r"`, since the reverse twin's
id is derived automatically. `circulating` is computed once: a road is part
of a roundabout ring either because the GeoJSON says so directly
(`"roundabout": true`) or, failing that, because *both* endpoints are
already-loaded nodes whose `zone` was `"roundabout"` — this is a fallback
inference for maps that mark the *nodes* as roundabout without redundantly
marking every ring segment. `graph.edges[id]` constructs the forward `Road`,
and — this is the key documented behavior from the header comment —
`if (!oneway)` constructs a *second* `Road` under `id + "_r"`, same
endpoints reversed, same length/speed/lanes, but always `oneway=false`
passed as its own directionality flag (each direction of a two-way street
is internally still a distinct one-directional `Road` object).

**Why:** representing a two-way street as two independent `Road` objects
rather than one bidirectional edge keeps `Road`'s own model simple — see
[`learn/02-core-graph-model.md`](02-core-graph-model.md) for why the graph
is directed. Deriving the roundabout flag from either an explicit tag or
inference from endpoint zones is a robustness choice for the two source
formats this loader has to support (hand-authored native maps likely tag
each ring segment; the OSM path below instead tags entire ways). The inline
comment (lines 125–128 in the source) explains the semantic payoff
directly: a road inside a ring must not stop for traffic ahead of it on the
*same* ring the way an ordinary crossing would — flagging it `is_internal`
is what tells the junction-arbitration logic in
[`learn/07-simulation-core.md`](07-simulation-core.md) to skip the usual
give-way rule there.

**Usage:** called once per `uts:kind=road` feature in pass 2. The `is_internal`
flag it sets is read by `JunctionArbiter` (see
[`learn/07-simulation-core.md`](07-simulation-core.md)) — this is one of the
few places outside the simulation core where the "junction overlap" /
give-way limitation is directly relevant: getting `is_internal` wrong on a
loaded map (e.g. missing roundabout tags) is exactly the kind of
data-quality bug that would produce visibly wrong arbitration at runtime,
distinct from the deeper algorithmic overlap limitation itself.

---

### `MapLoader.cpp` lines 147–202: `load_zone` and `load_building`

```cpp
void load_zone(TrafficGraph& graph, const json& props, const json& geom) {
    if (geom.value("type", "") != "Polygon") return;  // background region: skip if malformed
    const json& coords = geom["coordinates"];
    if (!coords.is_array() || coords.empty() || !coords[0].is_array()) return;

    double min_x = 1e18, min_y = 1e18, max_x = -1e18, max_y = -1e18;
    bool has_point = false;
    for (const auto& pt : coords[0]) {
        double x = 0.0, y = 0.0;
        if (!read_point(pt, x, y)) continue;
        min_x = std::min(min_x, x); max_x = std::max(max_x, x);
        min_y = std::min(min_y, y); max_y = std::max(max_y, y);
        has_point = true;
    }
    if (!has_point) return;
    graph.zones.push_back({str_prop(props, "name"), str_prop(props, "kind"),
                           min_x, min_y, max_x - min_x, max_y - min_y});
}
```

**Meaning:** rather than storing a zone's exact polygon, this reduces it to
its *axis-aligned bounding box* — it scans every point in the outer ring
(`coords[0]`), tracks running min/max x and y, and stores the zone as
`{name, kind, x=min_x, y=min_y, w=width, h=height}`. Any malformed shape
(wrong geometry type, empty coordinate array, no readable points at all)
is silently skipped by an early `return` rather than throwing.

**`load_building`** (lines 168–202) is structurally similar but keeps the
*actual* polygon rather than reducing to a box: it collects each `[x, y]`
point into a `footprint` vector, drops the last point if it duplicates the
first (GeoJSON polygon rings are closed — the first and last coordinate are
identical by spec — but this codebase's `Building` stores an open ring), and
requires at least 3 points to bother keeping it. It also handles a naming
collision: `buildings` is a map keyed by name, so if the same name is
already taken, it appends `" #2"`, `" #3"`, etc. until it finds a free key.
The optional `"node"` property is stored via `set_node_id`, which the
comment explains: it lets a `Vehicle` pick this building as a destination
(see [`learn/05-vehicles.md`](05-vehicles.md) for `Vehicle::request_route`).

**Why:** the class comment calls zones "display-only background regions" —
they're never routed through or collided with, only drawn, so collapsing
them to a bounding box is a deliberate simplification: the frontend only
ever needs to paint a rectangle, and a box is far cheaper to carry through
`map_json_string` than an arbitrary polygon. Buildings, by contrast, *do*
need their real footprint (for accurate rendering and, via the optional
node anchor, as a routable destination), so they keep full point lists.
Both functions skip-on-malformed rather than throw, unlike `load_node`/
`load_road` — a broken zone or building degrades the map cosmetically; a
broken node or road breaks routing correctness. That asymmetry in error
handling strictness tracks exactly how load-bearing each feature type is to
the simulation actually working.

**Usage:** `load_zone` results feed `graph.zones`, read straight back out by
`map_json_string`/`write_geojson` below and by the frontend renderer.
`load_building` results feed `graph.buildings`, consumed the same way plus
by `Vehicle::request_route` when a building is a trip's destination.

**Check yourself:**
- Why does `load_zone` reduce a polygon to a bounding box while
  `load_building` keeps the full point list? What would break if they
  swapped strategies?
- Trace what happens if two buildings in the same file both have `name=""`
  — walk through the `key` collision-avoidance loop by hand.
- Why is `load_road` strict (throws on bad input) while `load_zone`/
  `load_building` are lenient (skip on bad input)? What property of each
  feature type justifies the difference?

---

### `MapLoader.cpp` lines 204–338: the OSM ingestion header and helpers

```cpp
// ===========================================================================
// OSM flavor: derive the graph from a raw OpenStreetMap / Overpass export.
// Ported from the reference project and adapted to this repo's TrafficGraph.
// ===========================================================================

constexpr double kMetersPerDegLat = 110540.0;
constexpr double kMetersPerDegLon = 111320.0;
constexpr double kOriginMargin = 80.0;
constexpr double kSimplifyTolM = 3.0;
constexpr double kMetersPerVehicle = 20.0;
constexpr double kSignalSnapM = 25.0;
```

**Meaning:** this comment banner marks the start of a second, much larger
ingestion path, distinct from the native `load_node`/`load_road`/`load_zone`/
`load_building` functions above. It admits its own provenance — "ported from
the reference project" — meaning this logic predates this codebase and was
adapted rather than designed from scratch here. Five tuning constants
follow: two are geodesy constants (meters per degree of latitude/longitude,
used for a flat local projection — see the projection step below),
`kOriginMargin` is a fixed padding so the projected map doesn't start flush
against world-coordinate zero, `kSimplifyTolM` is the tolerance (in meters)
for a Douglas-Peucker curve-simplification pass, `kMetersPerVehicle` is a
jam-spacing constant used to derive road capacity from length, and
`kSignalSnapM` bounds how far a `traffic_signals` OSM node may be walked
along the road network before giving up on finding the junction it belongs
to (with a detailed comment on that value, discussed below).

**Why:** raw OSM data was never designed to be a traffic simulator's input
format — it's designed for maps and routing engines with very different
data models (ways instead of directed roads, ambiguous lane counts, signal
nodes placed on approach roads rather than on junctions themselves). This
section exists because supporting real city data (as opposed to only
hand-authored toy maps) means solving a list of translation problems: no
lon/lat-to-meters conversion is provided, ways aren't automatically split at
junctions, lane counts are often missing or split across two directions
combined, and traffic-signal placement is only approximately linked to the
junction it controls. Every constant here is a documented answer to one of
those specific translation problems, not an arbitrary tuning knob.

**Usage:** feeds into `load_osm` (below), the single entry point for this
entire code path, itself called only when `load_from_geojson_string`
determines (by absence of the `uts:kind` tag) that the input isn't a native
map.

---

### `MapLoader.cpp` lines 227–257: highway classification and lane/speed defaults

```cpp
bool is_drivable_highway(const std::string& hw) {
    static const std::unordered_set<std::string> kDrivable = {
        "motorway", "trunk", "primary", "secondary",
        "tertiary", "unclassified", "residential", "living_street",
        "service", "motorway_link", "trunk_link", "primary_link",
        "secondary_link", "tertiary_link",
    };
    return kDrivable.count(hw) > 0;
}

double default_speed_kmh(const std::string& hw) { ... }
int default_lanes_per_dir(const std::string& hw) { ... }
int capacity_for(int lanes_per_dir, double length_m) {
    return std::max(2, static_cast<int>(
                           std::lround(lanes_per_dir * length_m / kMetersPerVehicle)));
}
```

**Meaning:** `is_drivable_highway` checks OSM's `highway=*` tag against a
fixed allow-list of vehicle-drivable classes, deliberately excluding
pedestrian/cycle-only tags like `footway` or `cycleway` (the comment above
this section says as much: "Foot/cycle infrastructure is excluded").
`default_speed_kmh`/`default_lanes_per_dir` map a highway class to a
reasonable guess when OSM's own `maxspeed`/`lanes` tags are missing —
motorway fastest, service road slowest, motorway/trunk/primary get two
lanes per direction by default, everything else one. `capacity_for` derives
a road's vehicle capacity from its geometric length and lane count divided
by the jam-spacing constant (`kMetersPerVehicle = 20.0`), floored at a
minimum of 2 so even a very short segment can hold something.

**Why:** OSM data is famously incomplete for exactly these fields — many
ways have no `maxspeed` or `lanes` tag at all, since human mappers don't
always fill them in. Rather than fail the entire import over missing
optional tags, the loader falls back to editorially reasonable defaults
keyed by road class, which is standard practice for OSM-consuming routing
software. `capacity_for`'s formula is a physical model, not an arbitrary
number: "one vehicle roughly every 20 meters, per lane" is a plausible jam
density, so capacity scales naturally with how much road there physically
is.

**Usage:** feeds directly into the `OsmWay` construction inside `load_osm`
and, downstream, into the `Road` objects `add_edge` creates — the same
`capacity`/`speed`/`lanes` fields `load_road` (native path) reads straight
from the file.

---

### `MapLoader.cpp` lines 259–284: Douglas-Peucker line simplification

```cpp
double perp_distance(const Vec2& p, const Vec2& a, const Vec2& b) {
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double l2 = dx * dx + dy * dy;
    if (l2 <= 0.0) return std::hypot(p.x - a.x, p.y - a.y);
    double t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / l2;
    t = std::clamp(t, 0.0, 1.0);
    return std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

void simplify_keep(const std::vector<Vec2>& pts, std::size_t a, std::size_t b,
                   double tolerance, std::vector<bool>& keep) {
    if (b <= a + 1) return;
    double worst = -1.0;
    std::size_t worst_idx = a;
    for (std::size_t i = a + 1; i < b; ++i) {
        const double d = perp_distance(pts[i], pts[a], pts[b]);
        if (d > worst) { worst = d; worst_idx = i; }
    }
    if (worst > tolerance) {
        keep[worst_idx] = true;
        simplify_keep(pts, a, worst_idx, tolerance, keep);
        simplify_keep(pts, worst_idx, b, tolerance, keep);
    }
}
```

**Meaning:** `perp_distance` computes the perpendicular distance from point
`p` to the line *segment* `a`–`b` (not the infinite line): it projects `p`
onto the segment via a dot-product parameter `t`, clamps `t` to `[0, 1]` so
the projection can't fall outside the segment's actual endpoints, and
returns the Euclidean distance to that clamped projection point. `l2 <= 0.0`
guards against a degenerate zero-length segment (`a == b`), falling back to
plain point-to-point distance. `simplify_keep` is the classic recursive
Douglas-Peucker algorithm: given a range of points `pts[a..b]`, find the
point farthest (perpendicular distance) from the straight chord between the
endpoints; if that worst distance exceeds `tolerance`, mark that point as
"keep" and recurse on both halves; otherwise every point strictly between
`a` and `b` is considered close enough to the straight line to discard.

**Why:** OSM way geometry is densely sampled — a curving road might have a
GPS trace point every few meters — but this simulator only needs the
*shape* (for routing and rendering), not every sampled point; too many
points means unnecessarily many short `Road` segments and junction-like
waypoint nodes for what should be one smooth curve. Douglas-Peucker is the
standard algorithm for this exact tradeoff: it guarantees the simplified
line never deviates from the original by more than `tolerance`
(`kSimplifyTolM = 3.0` meters here), while discarding as many points as
possible within that bound.

**Usage:** called from `load_osm`'s way-splitting loop (below) on each
segment between junctions, to decide which intermediate vertices become
`Intersection` "waypoint" nodes and which are dropped entirely.

---

### `MapLoader.cpp` lines 286–338: `OsmWay` and OSM building projection

```cpp
struct OsmWay {
    std::string id;
    std::vector<std::size_t> verts;
    std::string name;
    std::string highway;
    double speed_kmh = 30.0;
    int lanes_per_dir = 1;
    bool oneway = false;
    bool roundabout = false;
};

template <typename ProjectFn>
void load_building_osm(TrafficGraph& graph, const json& props, const json& geom,
                       ProjectFn&& project) {
    ...
    auto ring_to_building = [&](const json& ring) { ... };
    if (type == "Polygon" && !geom["coordinates"].empty()) {
        ring_to_building(geom["coordinates"][0]);
    } else if (type == "MultiPolygon") {
        for (const auto& poly : geom["coordinates"]) {
            if (!poly.empty()) ring_to_building(poly[0]);
        }
    }
}
```

**Meaning:** `OsmWay` is a plain aggregate describing one parsed OSM road
segment before it's split into graph edges: its vertex indices into a
shared vertex pool (rather than owning coordinates directly), its
tags-derived name/highway class/speed/lane count, and two booleans for
directionality and roundabout membership. `load_building_osm` is a template
function parameterized on `ProjectFn` — a callable that converts a raw
lon/lat point into local projected meters — so the same building-extraction
logic works regardless of exactly how the caller chooses to project
coordinates (this is compile-time polymorphism via templates, distinct from
the `Strategy` pattern's runtime polymorphism covered in
[`learn/04-routing.md`](04-routing.md)).
It handles both GeoJSON `Polygon` (single ring, take the outer boundary
`geom["coordinates"][0]`) and `MultiPolygon` (a list of polygons, each
processed the same way) geometry types, since OSM buildings export as
either depending on complexity.

**Why:** storing `verts` as indices into a pool (rather than each `OsmWay`
holding its own `Vec2` copies) is a memory/dedup optimization — OSM ways
routinely share endpoints with neighboring ways (that's literally how a
road network connects), so a shared vertex pool means "is this the same
point as that other way's endpoint" becomes an index comparison instead of
a floating-point equality check. The `ProjectFn` template parameter avoids
either duplicating the projection math in two places or paying for a
`std::function` indirection on a per-point hot loop.

**Usage:** `OsmWay` instances are built up in `load_osm`'s way-reading loop
and consumed by the way-splitting loop right after. `load_building_osm` is
called once per OSM building feature near the end of `load_osm`.

**Check yourself:**
- Why does `perp_distance` clamp `t` to `[0, 1]` instead of measuring
  distance to the infinite line through `a` and `b`?
- What does storing `OsmWay::verts` as indices (instead of raw coordinates)
  save, and what would break if two ways with a shared endpoint got
  slightly different floating-point copies of that point instead?
- Why is `load_building_osm` a template on `ProjectFn` rather than taking a
  `std::function<Vec2(double,double)>`?

---

### `MapLoader.cpp` lines 340–404: `load_osm` — reading vertices and ways

```cpp
void load_osm(TrafficGraph& graph, const std::vector<const json*>& road_features,
              const std::vector<const json*>& building_features,
              const std::vector<const json*>& signal_features,
              const std::string& source) {
    if (road_features.empty()) {
        throw MapLoadError("No drivable roads (highway=*) found in " + source);
    }

    std::map<std::pair<long long, long long>, std::size_t> vertex_index;
    std::vector<Vec2> raw_verts;
    auto vertex_for = [&](double x, double y) {
        const auto key = std::make_pair(static_cast<long long>(std::llround(x * 1e7)),
                                        static_cast<long long>(std::llround(y * 1e7)));
        auto [it, inserted] = vertex_index.emplace(key, raw_verts.size());
        if (inserted) raw_verts.push_back({x, y});
        return it->second;
    };
    ...
}
```

**Meaning:** `load_osm` takes three pre-filtered lists of raw GeoJSON
feature pointers (road, building, and signal features — filtering happens
in `load_from_geojson_string`, covered below) rather than the whole document,
and immediately rejects empty input. `vertex_for` is the "shared vertex
pool" mechanism promised above: it rounds a lon/lat coordinate to 7 decimal
places (`x * 1e7`, then `llround` to an integer — OSM's own stated precision
is 1e-7 degrees, so this rounding doesn't lose real precision, it only
collapses floating-point representation noise), uses that as a `map` key,
and either returns the existing pool index for a coordinate seen before or
appends a new one. Structured bindings (`auto [it, inserted] = ...`) unpack
the pair `std::map::emplace` returns — the iterator to the (possibly
newly-inserted) entry, and whether insertion actually happened.

**Why:** two OSM ways that meet at an intersection will each report that
shared point with essentially — but not always bit-for-bit — identical
floating-point coordinates, depending on how the export tool serialized
them. Comparing raw doubles for equality is fragile; rounding to a fixed
integer grid (matching OSM's actual coordinate precision) makes "same point"
a reliable, exact integer-key lookup instead. This is the load-bearing trick
that lets the rest of `load_osm` treat "two ways share a vertex" as a simple
fact rather than a fuzzy geometric judgment call.

**Usage:** every vertex referenced by every way and every signal point is
funneled through `vertex_for`, so `raw_verts` ends up holding exactly one
entry per distinct real-world point, and all subsequent indices
(`w.verts`, `signal_verts`, etc.) refer consistently into that one array.

---

### `MapLoader.cpp` lines 360–404: parsing each way's tags

```cpp
for (const json* f : road_features) {
    const json& props = (*f)["properties"];
    const json& coords = (*f)["geometry"]["coordinates"];
    if (!coords.is_array() || coords.size() < 2) continue;

    OsmWay w;
    w.highway = str_prop(props, "highway");
    std::string osm_id = str_prop(props, "@id", str_prop(props, "id"));
    const auto slash = osm_id.find('/');
    if (slash != std::string::npos) osm_id = osm_id.substr(slash + 1);
    w.id = osm_id.empty() ? std::to_string(ways.size()) : osm_id;
    w.name = str_prop(props, "name");
    w.speed_kmh = num_prop(props, "maxspeed", default_speed_kmh(w.highway));

    const std::string oneway = str_prop(props, "oneway");
    const bool roundabout = str_prop(props, "junction") == "roundabout";
    w.roundabout = roundabout;
    const bool reversed = oneway == "-1";
    w.oneway = reversed || roundabout || oneway == "yes" || oneway == "1" ||
               oneway == "true";

    const int total_lanes = static_cast<int>(num_prop(props, "lanes", 0));
    if (total_lanes > 0) {
        w.lanes_per_dir = w.oneway ? total_lanes : (total_lanes + 1) / 2;
    } else {
        w.lanes_per_dir = default_lanes_per_dir(w.highway);
    }
    w.lanes_per_dir = std::clamp(w.lanes_per_dir, 1, 4);

    for (const auto& pt : coords) {
        double x = 0.0, y = 0.0;
        if (read_point(pt, x, y)) w.verts.push_back(vertex_for(x, y));
    }
    if (reversed) std::reverse(w.verts.begin(), w.verts.end());
    if (w.verts.size() >= 2) ways.push_back(std::move(w));
}
```

**Meaning:** for each road feature, it extracts an id — preferring `@id`
(Overpass's own id-tagging convention, e.g. `"way/123"`) over a plain `id`
property, then strips any `"way/"` prefix via `find('/')`/`substr`, falling
back to a positional index if neither is present. `oneway` is read as a
raw string because OSM encodes it inconsistently: `"-1"` means one-way but
*reversed* relative to the geometry's stored point order, `"1"`/`"yes"`/
`"true"` mean one-way in the stored order, and a `junction=roundabout` tag
implies one-way even without an explicit `oneway` tag (rings only make
sense circulating one direction). Lane count similarly needs interpretation:
OSM's `lanes` tag is a *total* across both directions on a two-way street,
so `(total_lanes + 1) / 2` (integer division, rounding up) splits it in
half per direction — the `+1` before dividing is what makes an odd total
(e.g. 3 lanes) round up rather than down for at least one direction. The
result is clamped to `[1, 4]` lanes per direction as a sanity bound. Every
coordinate is looked up through `vertex_for` and pushed into `w.verts`; if
`reversed`, the whole vertex list is flipped so the way's direction matches
what `oneway=-1` actually means; ways with fewer than 2 vertices (degenerate)
are silently dropped.

**Why:** every one of these branches is a direct translation of an OSM
tagging quirk into this simulator's cleaner model (a `Road` with an
unambiguous `oneway` bool and a lane count *per direction*, not total).
Getting `oneway=-1` wrong (forgetting to reverse the vertex order) would
silently build every such road backwards — vehicles routed "from A to B"
would actually be sent the wrong way down a real one-way street, a subtle,
hard-to-notice bug rather than a crash, which is presumably why this
comment-dense handling exists.

**Usage:** the resulting `ways` vector feeds directly into the
junction-detection and way-splitting logic immediately following.

**Check yourself:**
- Walk through what `w.oneway` and `w.verts`' order end up being for a way
  tagged `oneway=-1` with three points `[P0, P1, P2]` in the GeoJSON. Which
  physical direction does the resulting `Road` represent?
- Why does `lanes_per_dir` use `(total_lanes + 1) / 2` instead of
  `total_lanes / 2`? Try it with `total_lanes = 3` and `total_lanes = 4` by
  hand.

---

### `MapLoader.cpp` lines 405–460: signal vertices and the lon/lat projection

```cpp
std::unordered_set<std::size_t> signal_verts;
for (const json* f : signal_features) {
    const json& c = (*f)["geometry"]["coordinates"];
    double x = 0.0, y = 0.0;
    if (!read_point(c, x, y)) continue;
    const auto key = std::make_pair(static_cast<long long>(std::llround(x * 1e7)),
                                    static_cast<long long>(std::llround(y * 1e7)));
    auto it = vertex_index.find(key);
    if (it != vertex_index.end()) signal_verts.insert(it->second);
}

double min_x = 1e18, max_x = -1e18, min_y = 1e18, max_y = -1e18;
for (const Vec2& v : raw_verts) { ... }
const bool lonlat = is_lonlat_range(min_x, min_y) && is_lonlat_range(max_x, max_y);
const double lon0 = (min_x + max_x) / 2.0, lat0 = (min_y + max_y) / 2.0;
const double cos_lat0 = std::cos(lat0 * 3.14159265358979323846 / 180.0);
auto project_base = [&](double x, double y) -> Vec2 {
    if (!lonlat) return {x, y};
    return {(x - lon0) * kMetersPerDegLon * cos_lat0,
            (lat0 - y) * kMetersPerDegLat};
};
```

**Meaning:** signal points are matched into the *same* vertex pool built
from road ways, using the identical rounding-to-1e-7 key — if an OSM
`traffic_signals` node's coordinates exactly coincide (after rounding) with
a point already on some road's geometry, its pool index is recorded in
`signal_verts`; otherwise it's silently dropped (unmatched signals get
handled much later with a fallback strategy, discussed below). The
projection step first computes the bounding box of every vertex in the
pool, then `is_lonlat_range` checks whether that whole box plausibly looks
like longitude/latitude (`x` in `[-180, 180]`, `y` in `[-90, 90]`) — if not,
`lonlat` is `false` and the data is assumed to already be in projected
meters, so `project_base` is a no-op passthrough. If it *is* geographic,
`lon0`/`lat0` are the bounding box's center, and `cos_lat0` is the cosine of
the center latitude in radians (hand-computed `3.14159265358979323846 / 180.0`
rather than using a `<numbers>` constant, presumably for portability to
compilers without full C++20 `<numbers>` support). `project_base` then
applies a standard **equirectangular projection**: longitude difference from
the center scaled by meters-per-degree-longitude *times* `cos(lat0)` (because
a degree of longitude covers less physical distance the farther from the
equator you are — this is the projection's one correction for the earth's
curvature), and latitude difference from the center scaled by
meters-per-degree-latitude, with `lat0 - y` (not `y - lat0`) flipping the
sign so that increasing latitude (further north) maps to a *smaller* screen
y — i.e., "north stays up on screen," per the inline comment.

**Why:** an equirectangular projection is a deliberately simple choice — it
distorts distances increasingly far from the reference point, but for a
single city-scale extract (the docstring above `kMetersPerDegLat` says "fine
for city-scale extracts") the distortion is negligible, and it's vastly
cheaper and simpler than a proper conic or Mercator-family projection. The
`is_lonlat_range` auto-detection means this same function correctly handles
both a raw lon/lat OSM export and a GeoJSON export that's already been
pre-projected into meters by some other tool, without a config flag telling
it which.

**Usage:** `project_base` (extended by a subsequent shift step, not shown
above, that translates every vertex so the map's minimum corner sits at
`kOriginMargin`) is the function every subsequent x/y coordinate in this OSM
path is put through — way vertices, signal positions, and building
footprints all pass through the same projection, guaranteeing they end up
in one consistent world-meters coordinate space.

**Check yourself:**
- Why does the projection multiply the longitude term by `cos(lat0)` but
  not the latitude term?
- What would go wrong if a mixed dataset (already in meters) were run
  through `project_base` as though it were lon/lat? How does `is_lonlat_range`
  prevent that specific failure?

---

### `MapLoader.cpp` lines 446–548: junction detection and way splitting

This is the structurally hardest part of the OSM path: OSM ways are long,
multi-block polylines with no concept of "intersection" baked in, while this
simulator's graph needs a `Road` edge between every pair of adjacent
junctions.

```cpp
std::vector<int> use_count(verts.size(), 0);
std::vector<bool> is_endpoint(verts.size(), false);
for (const OsmWay& w : ways) {
    for (std::size_t v : w.verts) {
        ++use_count[v];
        if (!w.name.empty()) junction_names[v].insert(w.name);
    }
    is_endpoint[w.verts.front()] = true;
    is_endpoint[w.verts.back()] = true;
}
auto is_junction = [&](std::size_t v) {
    return is_endpoint[v] || use_count[v] >= 2;
};
```

**Meaning:** `use_count[v]` tallies how many ways touch each vertex.
`is_endpoint[v]` separately marks a vertex as a way's own first/last point
(a way can dead-end at a vertex that no other way touches, which still has
to become a junction node because it's where the road physically stops).
`is_junction` combines both: a vertex counts as a junction if it's an
endpoint of some way, *or* if two or more ways share it (a mid-way point
where two roads cross or merge).

Then the way-splitting loop (lines 502–548) walks each way's vertex
sequence, accumulating a "segment" until it hits a junction vertex, applies
Douglas-Peucker simplification to that segment (forcing any signal vertex to
survive simplification regardless of the tolerance check, per the inline
comment — "the only anchor saying where the light belongs"), and emits one
graph `Road` edge per surviving simplified sub-segment via `add_edge`, using
lazily-created `junction_node_id`/`waypoint_node_id` helpers — a true
junction gets a permanent numbered id (`"n0"`, `"n1"`, ...), while a merely
shape-preserving interior point becomes a `"wp0"`, `"wp1"`, ... node tagged
`zone = "waypoint"`.

**Why:** the junction/waypoint distinction is the crux of the whole
translation from "OSM way" to "this simulator's graph": a `Road`'s `source`/
`target` must be actual decision points (real intersections), but an OSM
way's raw geometry has far more points than that, most of which just trace
a curve. Splitting *only* at junction vertices (not every vertex) keeps the
graph's node count proportional to real intersections rather than to GPS
sample density, while Douglas-Peucker on each segment keeps the visual
curve shape without needing every original sample point as a waypoint node.
Forcing signal vertices to survive simplification is necessary because a
signal that got smoothed away would have nothing left in the graph to
anchor it to — the signal-matching pass described next depends on that
vertex still being a real node id.

**Usage:** the output of this loop is what populates `graph.intersections`
(junction and waypoint nodes) and `graph.edges` for the entire OSM-derived
map — this is the graph the rest of the simulator operates on identically
to a natively-authored one.

**Check yourself:**
- Why does a way's own first/last vertex always count as a junction, even
  if no other way touches it?
- What would go wrong for a `traffic_signals` OSM node if this loop's
  simplification step did *not* special-case keeping signal vertices?

---

### `MapLoader.cpp` lines 550–653: roundabout marking and strongly-connected-component pruning

```cpp
{
    std::unordered_map<Intersection*, std::pair<int, int>> ring;
    for (const auto& [id, road] : graph.edges) {
        if (!road) continue;
        for (Intersection* n : {road->source, road->target}) {
            if (!n) continue;
            auto& c = ring[n];
            c.second++;
            if (road->is_internal) c.first++;
        }
    }
    for (const auto& [node, c] : ring) {
        if (c.first > 0 && c.first == c.second) node->is_roundabout = true;
    }
}
```

**Meaning:** for every node, this counts how many incident roads are
`is_internal` (circulating, roundabout-tagged) versus how many total roads
touch it. A node is only marked `is_roundabout = true` if *every* road
touching it is internal (`c.first == c.second`, and at least one exists).
The comment above explains why this specific rule: a node where an approach
road (not circulating) meets the ring stays an ordinary crossing on
purpose, because entering traffic genuinely has to give way there, and only
a "crossing" arbitrates that — a node fully surrounded by circulating roads
is a seam OSM happened to place between two consecutive ring segments,
where nothing actually crosses.

The following block (lines 575–635) runs a from-scratch **Kosaraju's
algorithm** (two DFS passes — one to compute a finishing-order stack, one
on the transposed graph to peel off strongly-connected components) to find
the single largest strongly-connected component of the road network, then
deletes every edge and node *outside* it.

**Why:** clipped OSM extracts (a rectangular bounding-box cut out of a
larger city) routinely leave dangling one-way fragments at the boundary —
a one-way road that enters the clipped area but whose matching return path
was cut off outside the box becomes an unroutable "trap": a vehicle could
enter it but never leave. Rather than let routing fail unpredictably on
whichever node happens to be stranded, the loader proactively keeps only
the *single* largest strongly-connected component (where every node can
reach every other node, forward and backward) and discards the rest,
guaranteeing every remaining node is genuinely routable to and from every
other. This is the same kind of "reactive, not proactive" tradeoff the
project brief documents about the simulator's gridlock handling in general
— here applied at map-load time to a different problem (connectivity, not
gridlock), but it fits the same pattern of accepting an occasional
"cluster gets dropped and never appears" cost in exchange for avoiding the
much worse "stranded vehicle" failure mode.

**Usage:** the diagnostic line
`"[map] pruned %zu node(s) outside the largest connected road network..."`
prints to stderr whenever this pruning actually removes something — a
useful signal when importing a real city extract that its bounding box cut
through the road network.

**Check yourself:**
- Why does the roundabout-marking rule require *every* incident road to be
  circulating, rather than *any*?
- Why strongly-connected components specifically, rather than simple
  (undirected) connected components? What would a merely-connected-but-not-
  strongly-connected node look like in a directed road graph, and why is it
  still a problem?

---

### `MapLoader.cpp` lines 654–813: signal matching, phase demotion, and buildings

```cpp
// OSM tags highway=traffic_signals on the APPROACH road a few metres before
// the intersection -- one node per arm -- not on the intersection node
// itself. Measured on a Ho Chi Minh City extract: of 223 signal nodes only
// 18 landed exactly on a junction; 205 landed mid-way, where they become
// "waypoint" nodes that assign_signal_groups() rejects outright. Demanding
// an exact hit therefore threw away ~92% of the city's real lights.
```

**Meaning:** this comment states a concrete, measured motivation for the
walk-to-junction algorithm that follows: a naive "signal node must coincide
exactly with a junction node" rule discarded 92% of real signals in a test
city extract, because OSM deliberately places `traffic_signals` points on
the *approach* road, not the junction itself. The code that follows
performs a bounded Dijkstra-style walk (capped at `kSignalSnapM = 25` meters)
outward from each signal vertex, expanding only through waypoint nodes,
stopping at the first true junction reached — the parent-pointer chain
recovered from that walk identifies exactly which `Road` (`arm`) the signal
actually sits on. It then calls `graph.signalize(best->name)` (see
[`learn/03-traffic-lights.md`](03-traffic-lights.md)) only if the junction
isn't already signalized, and only *demotes* — resets `signal_group = -1` —
approach roads that never had a matching signal node walked to them,
because `signalize()` by default assigns a phase group to *every* approach,
which would incorrectly stop traffic on arms the real city leaves running.

**Why:** this whole section is a worked example of the broader lesson the
comment states outright: exact-match heuristics against real-world OSM data
systematically fail because OSM's tagging conventions don't align with a
simulator's idealized model (a signal belongs to a junction, not an
approach road, in this project's own native format). The 25-meter snap
distance is itself justified in the constant's own comment as wide enough
to reach across a multi-lane approach but short enough not to overshoot
into the *next* intersection on a typical ~27-meter city block — a
concrete, measured tuning choice, not an arbitrary round number.

**Usage:** feeds directly into `TrafficLight`/junction signalization (see
[`learn/03-traffic-lights.md`](03-traffic-lights.md)); diagnostic counts
(`signalized`, `unmatched`, `refused`, `demoted`) are printed to stderr for
visibility into how much of a given import's signal data was actually
recoverable.

Buildings (lines 814–817) are handled last and simply: iterate
`building_features`, call `load_building_osm` per feature with the same
`project` lambda used for roads, so building footprints land in the same
coordinate space as the road network.

**Check yourself:**
- Why does the signal-matching walk stop at the *first* true junction
  reached, rather than considering all junctions within `kSignalSnapM`?
- Explain in your own words why an approach road with no matching OSM
  signal node gets *demoted* rather than just left unassigned by default.

---

### `MapLoader.cpp` lines 822–929: `load_from_geojson`/`load_from_geojson_string` — the format-detection entry point

```cpp
void MapLoader::load_from_geojson_string(TrafficGraph& graph, const std::string& text,
                                         const std::string& source_name) {
    json root;
    try {
        root = json::parse(text);
    } catch (const json::exception& e) {
        throw MapLoadError("malformed JSON in " + source_name + ": " + e.what());
    }
    if (!root.is_object() || root.value("type", "") != "FeatureCollection" ||
        !root.contains("features") || !root["features"].is_array()) {
        throw MapLoadError(source_name + " is not a GeoJSON FeatureCollection");
    }

    std::vector<const json*> features;
    std::vector<const json*> osm_roads, osm_buildings, osm_signals;
    bool native = root.value("generator", "") == "urban-traffic-simulator";
    for (const auto& f : root["features"]) {
        ...
        if (str_prop(props, "uts:kind") == "node") native = true;
        ...
    }

    if (!native) {
        load_osm(graph, osm_roads, osm_buildings, osm_signals, source_name);
        return;
    }

    // native flavor: two passes over the uts:kind features
    for (const json* f : features) { ... load_node ... }
    if (graph.intersections.empty()) {
        throw MapLoadError(source_name + " contains no nodes (empty graph)");
    }
    for (const json* f : features) { ... load_road / load_zone / load_building ... }
    for (auto& [id, node] : graph.intersections) { ... assign_signal_groups ... }
    graph.reindex();
}
```

**Meaning:** parsing is wrapped in a `try`/`catch` translating
`nlohmann::json`'s own `json::exception` into this project's `MapLoadError`,
so callers never need to know or catch a third-party exception type.
Structural validation follows (must be an object, `type == "FeatureCollection"`,
`features` must be a present array) before any feature is touched. `native`
is determined two ways: fast-path, the document's own `"generator"` field
(only `write_geojson`, covered below, stamps this) says
`"urban-traffic-simulator"`; otherwise, the loop over every feature checks
whether *any* feature carries `uts:kind = "node"`, in which case `native` is
set `true` mid-loop. Every feature is also simultaneously classified for the
*OSM* path (`osm_roads`/`osm_buildings`/`osm_signals`), even though those
lists are only used if `native` ends up false — the classification is cheap
and doing it in the same pass avoids iterating the feature list twice. If
`native` is false after the full scan, `load_osm` runs and the function
returns immediately. Otherwise, the native format's three-pass structure
runs: nodes first (raising `MapLoadError` if none exist at all — an empty
graph is treated as a load failure, not silently accepted), then roads/
zones/buildings (which need nodes to already exist), then a final pass
assigning signal phase groups now that every road (and therefore every
approach) exists. `graph.reindex()` at the very end rebuilds whatever
internal per-node/per-road index caches the graph maintains (see
[`learn/02-core-graph-model.md`](02-core-graph-model.md) and the reindex
discussion in `MapEditor` below).

**Why:** the `"generator"` field fast-path exists specifically to avoid a
subtle misclassification bug spelled out in the inline comment: a map saved
by this project's own `write_geojson` with *zero* nodes in it (an
edge case, but a legitimate one — an empty city someone is about to build
from scratch) has nothing to sniff for `uts:kind = "node"`, so without the
generator tag it would be mistaken for an OSM export and fail with a
confusing "no drivable roads" error instead of the more honest "graph is
empty." Doing the OSM-vs-native decision from feature *content* rather than
a file naming convention or explicit CLI flag is what lets one function
handle both a hand-authored map and a raw Overpass export transparently.

**Usage:** `load_from_geojson` (a thin wrapper, below) is called from
`TrafficSimulator::load()`; `load_from_geojson_string` directly backs the
"import an uploaded map" server endpoint (see
[`learn/07-simulation-core.md`](07-simulation-core.md) /
[`learn/09-server-and-entrypoint.md`](09-server-and-entrypoint.md)).

---

### `MapLoader.cpp` lines 824–834 and 931–950: `load_from_geojson` and `validate`

```cpp
void MapLoader::load_from_geojson(TrafficGraph& graph, const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) {
        throw MapLoadError("cannot open map file: " + path);
    }
    std::ostringstream buffer;
    buffer << file.rdbuf();
    load_from_geojson_string(graph, buffer.str(), path);
}

std::string MapLoader::validate(const TrafficGraph& graph) {
    if (graph.intersections.empty()) return "graph is empty";

    const std::string origin = graph.intersections.begin()->first;
    std::unordered_set<std::string> reached{origin};
    std::vector<const Intersection*> frontier{graph.intersections.begin()->second.get()};
    for (std::size_t head = 0; head < frontier.size(); ++head) {
        for (const Road* road : frontier[head]->outgoing_edges) {
            if (reached.insert(road->target->name).second) {
                frontier.push_back(road->target);
            }
        }
    }
    const std::size_t unreachable = graph.intersections.size() - reached.size();
    if (unreachable > 0) {
        return std::to_string(unreachable) + " node(s) unreachable from '" + origin + "'";
    }
    return "";
}
```

**Meaning:** `load_from_geojson` opens the file in binary mode (avoiding
platform-specific text-mode newline translation, which could otherwise
subtly corrupt byte offsets or content on Windows), checks `is_open()`
explicitly (the comment in the header explains why: MinGW's `operator!`
overload doesn't reliably report a missing file until a read is attempted),
slurps the entire file into a string via `buffer << file.rdbuf()`, and
delegates to `load_from_geojson_string`. `validate` runs a single
breadth-first search — `frontier` grows by index (`head` walking forward
through a vector that's still being appended to, a common
allocation-free BFS idiom that avoids a separate queue container) — from an
arbitrary origin node, following only `outgoing_edges`, and reports however
many nodes were never reached as a plain descriptive string; an empty
string means "no problem found."

**Why:** `validate` is deliberately a *warning*, not a hard failure — it
returns a string rather than throwing, which is a caller-decides-severity
design: an unreachable node might be a legitimate isolated cul-de-sac in a
real city (not necessarily a data error), so the loader itself doesn't
refuse to load such a map, but callers (like the "import" endpoint) can
choose to surface the warning to whoever's uploading the file.

**Usage:** the import-map server endpoint (see
[`learn/07-simulation-core.md`](07-simulation-core.md)) calls `validate` on
a freshly-loaded candidate graph before accepting it, per the grep evidence
in this module's introduction (`MapLoader::validate(*candidate)`).

**Check yourself:**
- Why does `load_from_geojson` check `file.is_open()` explicitly instead of
  just `if (!file)`?
- Why does `validate` return a descriptive string instead of a `bool`, and
  why doesn't an unreachable node make the *load itself* fail?

---

### `MapLoader.cpp` lines 952–1137: serialization — `map_json_string`, `write_map_json`, `write_geojson`

```cpp
std::string MapLoader::map_json_string(const TrafficGraph& graph) {
    json out;
    out["intersections"] = json::array();
    ...
    for (const auto& [id, road] : graph.edges) {
        if (id.size() > 2 && id.compare(id.size() - 2, 2, "_r") == 0 &&
            graph.edges.count(id.substr(0, id.size() - 2))) {
            continue;
        }
        out["roads"].push_back({
            {"id", id}, {"from", road->source->name}, {"to", road->target->name},
            {"length", road->length}, {"lanes", road->get_lanes()},
            {"speed_kmh", road->get_speed_limit()}, {"oneway", road->is_one_way},
            {"base_index", road->base_index},
        });
    }
    ...
    return out.dump();
}
```

**Meaning:** this is the one place in `MapLoader.cpp` that builds output
using `nlohmann::json`'s DOM API (`json out; out["roads"] = json::array();
out["roads"].push_back({...})`) rather than hand-written string
concatenation — a deliberate contrast with `Json.hpp`'s "avoid the DOM in
hot paths" philosophy, justified because this function runs once per map
load/edit, not once per tick. The reverse-twin skip (`id + "_r"` check) is
the same "emit a two-way street once" pattern seen in `write_geojson`
(below): if `id` ends in `"_r"` and its base id (without the suffix) is
also present in `graph.edges`, this iteration is the reverse twin of an
edge already emitted, so it's skipped. `base_index` is called out by an
inline comment as existing specifically so the frontend can map a
per-vehicle index back to a road id via a lookup table, without doubling
payload size by separately emitting the `_r` twin's own base_index (the
frontend synthesizes that side client-side already, since it shares the
same `base_index`). `out.dump()` (not `out.dump(2)` or similar) is called
out as deliberately compact — "city-scale maps are big, skip
pretty-printing," trading human-readability for payload size.

`write_map_json` (below) is a two-line wrapper: call `map_json_string`, then
write the result to a file, translating a failed `ofstream` into
`MapLoadError`. `write_geojson` is the more involved sibling: it rebuilds
the *native* GeoJSON format (with the `"generator": "urban-traffic-simulator"`
stamp that `load_from_geojson_string` checks for) by walking
`graph.intersections` (writing signal timing back out if the node has a
light), `graph.edges` (same two-way-once skip, re-attaching `"roundabout": road->is_internal`
with an inline comment explaining that omitting this would demote a saved
roundabout back to an ordinary one-way street on reload — a direct
round-trip-fidelity requirement), `graph.buildings`, and `graph.zones` in
turn.

**Why:** the split between `map_json_string`/`write_map_json` (display
format) and `write_geojson` (native/reloadable format) is the same design
tension covered in the header walkthrough above — display JSON needs to be
lightweight and frontend-shaped; native GeoJSON needs to be lossless enough
that `load_from_geojson` can reconstruct the exact same graph (signal
timing, roundabout status, lane counts, building anchors) from what
`write_geojson` writes. The explicit `"roundabout"` round-trip comment is a
concrete example of a property that's easy to forget when writing a
serializer and expensive to get wrong (silently breaking roundabout
behavior on every reload of a saved city).

**Usage:** `map_json_string` backs `GET /api/map` per the grep evidence
above. `write_geojson` backs "save this map," reachable through
`TrafficSimulator`'s live-editing forwarders (see
[`learn/07-simulation-core.md`](07-simulation-core.md)).

**Check yourself:**
- Why does `map_json_string` use the `nlohmann::json` DOM builder API while
  the rest of this file's *reading* side avoids it? Is that inconsistent
  with `Json.hpp`'s stated rationale, or consistent with it? (Hint: how
  often does each of these functions run, relative to a tick.)
- What concretely breaks on the next `load_from_geojson` call if
  `write_geojson` forgot to write the `"roundabout"` property?

---

## `cpp_backend/include/simulation/MapEditor.hpp` and `cpp_backend/src/simulation/MapEditor.cpp`

### `MapEditor.hpp` lines 1–17: the class-level design rationale

```cpp
#pragma once
/*
MapEditor.hpp

- Owns live map editing: adding/removing junctions, roads and buildings,
  splitting a road under traffic, and re-tuning an existing road or light.
  Every method here assumes the caller (TrafficSimulator) already holds its
  simulation mutex -- this class has no lock of its own, the same convention
  TrafficSimulator's own "_locked" helpers already use.

- Split out of TrafficSimulator so structural graph edits have a class
  boundary of their own, matching the assignment's suggested component
  split. It is handed references to the graph pointer, the vehicle fleet and
  the event controller rather than owning any of them: editing the map can
  despawn vehicles standing on what got deleted and has to ask survivors to
  replan, which needs the same fleet TrafficSimulator drives every tick.
*/
```

**Meaning:** this comment establishes two facts up front, before any code:
(1) `MapEditor` has *no lock of its own* — thread-safety is entirely the
caller's responsibility, matching a convention already established
elsewhere in `TrafficSimulator` (its `_locked`-suffixed private helpers, see
[`learn/07-simulation-core.md`](07-simulation-core.md)); (2) `MapEditor`
doesn't *own* the graph, the vehicle fleet, or the event controller — it's
handed references to state that `TrafficSimulator` actually owns, because
editing the map (deleting a road a vehicle is currently on, for instance)
inherently needs to reach into that same live vehicle fleet the tick loop
drives.

**Why:** this is a direct instance of the Single Responsibility Principle
applied to a class that had grown structural-editing logic mixed into
`TrafficSimulator` itself — the comment explicitly frames this as "matching
the assignment's suggested component split," i.e. this is a refactor done
for architectural cleanliness, not because the code needed new behavior.
Choosing references over ownership (rather than, say, `MapEditor` owning
its own copy of the graph) is the only design that makes sense here: a copy
would immediately diverge from the live simulation state the moment either
side edited anything, so shared, single-source-of-truth state accessed by
reference is required, not just convenient.

**Usage:** `TrafficSimulator` holds a `MapEditor` as a member
(`map_editor_`, constructed with references into `TrafficSimulator`'s own
`graph_`, `vehicles_`, `controller_`, `next_id_` — confirmed by the grep at
the top of this module) and forwards its own public map-editing API to
`MapEditor`'s methods after acquiring its mutex, per the "Locking stays
here" comment found in `TrafficSimulator.cpp` (see
[`learn/07-simulation-core.md`](07-simulation-core.md)).

---

### `MapEditor.hpp` lines 19–75: constructor, public API, and the reference members

```cpp
class MapEditor {
public:
    MapEditor(TrafficGraph*& graph, std::vector<std::unique_ptr<Vehicle>>& vehicles,
             TrafficController& controller, int& next_id);

    void reindex();
    void invalidate_routes();

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

    const std::vector<std::string>& node_ids() const { return node_ids_; }
    const std::vector<std::string>& road_ids() const { return road_ids_; }

private:
    void destroy_vehicle(Vehicle* v);
    bool remove_road_locked(const std::string& id);

    TrafficGraph*& graph_;
    std::vector<std::unique_ptr<Vehicle>>& vehicles_;
    TrafficController& controller_;
    int& next_id_;

    std::vector<std::string> node_ids_;
    std::vector<std::string> road_ids_;
};
```

**Meaning:** the constructor's first parameter, `TrafficGraph*& graph`, is a
*reference to a pointer* — not a reference to a graph, and not a plain
pointer. This matters: `TrafficSimulator` can reassign which `TrafficGraph`
object `graph_` points at (loading a new map replaces the whole graph), and
because `MapEditor` holds a reference to that very pointer variable rather
than a snapshot of its current value, `MapEditor` automatically sees any
later reassignment without needing to be re-constructed or manually
re-pointed. `vehicles_`/`controller_`/`next_id_` are ordinary references —
`vehicles_` because `MapEditor` needs to erase entries directly (a vehicle
standing on a deleted road), `controller_` so it can detach a destroyed
vehicle from the Observer subject (see
[`learn/06-events-observer.md`](06-events-observer.md)), `next_id_`
as a shared counter for generating fresh, collision-free ids across every
kind of edit (`"j17"`, `"r42"`, ...). The public methods split into two
groups: structural mutators that return an id string on success or `""` on
failure (`add_junction_at`, `split_road_near`, `add_road_between`,
`add_building_at`), and simpler `bool`-returning mutators/queries for
existing elements (`remove_road`, `remove_intersection`, `update_road`,
`set_light`, `remove_building`). `node_ids()`/`road_ids()` expose cached id
lists as `const&`, kept current by `reindex()`.

**Why:** `TrafficGraph*&` (reference-to-pointer) versus `TrafficGraph&`
(reference-to-object) is a real, deliberate distinction here, not a
stylistic accident — the comment block above already told you why: the
graph itself gets replaced wholesale on a map reload, and `MapEditor` needs
to always operate on *whichever* graph is currently live, not the one that
existed at construction time. Returning `""`/`false` on failure rather than
throwing is consistent with this being a runtime user-facing editing API
(driven by clicks in the frontend map editor) rather than a load-time
parser — a user clicking to add a road between two nodes that don't exist
is an ordinary, expected outcome to handle gracefully, not an exceptional
one to unwind the stack over.

**Usage:** `node_ids()`/`road_ids()` back "pick a random spawn/destination"
logic elsewhere in `TrafficSimulator` (see
[`learn/07-simulation-core.md`](07-simulation-core.md)) — vehicle spawning
needs a cheap way to pick an existing node/road id without walking the
graph's maps every time.

**Check yourself:**
- Why is `graph` taken as `TrafficGraph*&` instead of `TrafficGraph&` or
  plain `TrafficGraph*`? What concrete bug would `TrafficGraph&` introduce
  after a map reload?
- Why do the structural-creation methods return an id string (empty on
  failure) while the structural-removal methods return `bool`?

---

### `MapEditor.cpp` lines 17–36: `reindex` and `invalidate_routes`

```cpp
void MapEditor::reindex() {
    node_ids_.clear();
    road_ids_.clear();
    if (!graph_) return;
    for (const auto& [id, node] : graph_->intersections) node_ids_.push_back(id);
    for (const auto& [id, road] : graph_->edges) road_ids_.push_back(id);
    graph_->reindex();
}

void MapEditor::invalidate_routes() {
    for (auto& v : vehicles_) v->request_reroute();
}
```

**Meaning:** `MapEditor::reindex()` rebuilds its own two cached id lists
from scratch by walking `graph_->intersections`/`graph_->edges` (both
presumably `std::map`/`std::unordered_map`, given the structured-binding
`[id, node]` iteration), then — critically — also calls `graph_->reindex()`,
delegating to the graph's *own* separate reindexing (covered in
[`learn/02-core-graph-model.md`](02-core-graph-model.md)). The inline
comment explains why this second call matters: without it, a newly created
node or road keeps whatever default index (likely `0`) it was constructed
with, silently colliding with — "aliasing" — whatever else already occupies
that slot in `JunctionArbiter`'s per-node data structures (see
[`learn/07-simulation-core.md`](07-simulation-core.md)). `invalidate_routes()`
iterates every live vehicle and calls `request_reroute()` on each — not a
`TrafficNotification` broadcast through the Observer system (see
[`learn/06-events-observer.md`](06-events-observer.md)), a direct
per-vehicle call instead, because the comment explains a structural graph
edit isn't one of that system's existing event *kinds*, and inventing a new
`Kind` just for this would be more machinery than the problem needs.

**Why:** the two-level reindex (`MapEditor`'s own id-list cache, plus the
graph's internal index) exists because two *different* consumers need
different things kept current: `MapEditor` itself needs flat lists for
"pick something existing" queries, while `TrafficGraph`/`JunctionArbiter`
need a dense internal indexing scheme (likely array-based, given the
"aliases whatever already owns that slot" wording) for O(1) lookup during
the tick loop. Choosing direct per-vehicle notification over the Observer
event system for route invalidation is a pragmatic call: a structural edit
is fundamentally different from a traffic *event* (an accident, a closure)
that other systems care about observing — it's specifically and only
relevant to routing, so it's handled as directly as possible.

**Usage:** every structural mutator below (`add_junction_at`,
`split_road_near`, `add_road_between`, `remove_road`, `remove_intersection`)
calls `reindex()`; every mutator that changes what the graph looks like to a
route-planner (all of the above, plus `update_road`) calls
`invalidate_routes()`. `add_building_at`/`remove_building` call neither,
because buildings aren't part of the routable graph (see
[`learn/02-core-graph-model.md`](02-core-graph-model.md) — buildings are
destinations *anchored to* a node, not nodes themselves).

**Check yourself:**
- Why does `MapEditor::reindex()` need to call `graph_->reindex()` in
  addition to rebuilding its own `node_ids_`/`road_ids_`? What's the
  concrete failure if it only did the latter?
- Why doesn't `invalidate_routes()` go through `TrafficController`'s
  Observer mechanism the way other traffic events do?

---

### `MapEditor.cpp` lines 38–75: destroying vehicles and dropping roads

```cpp
void MapEditor::destroy_vehicle(Vehicle* v) {
    if (!v) return;
    if (v->current_road) v->current_road->remove_vehicle(v);
    controller_.detach(v);
}

bool MapEditor::remove_road_locked(const std::string& id) {
    if (!graph_) return false;
    bool removed = false;
    for (const std::string& rid : {id, id + "_r"}) {
        auto it = graph_->edges.find(rid);
        if (it == graph_->edges.end()) continue;
        Road* road = it->second.get();

        for (auto vit = vehicles_.begin(); vit != vehicles_.end();) {
            if ((*vit)->current_road == road) {
                (*vit)->current_road = nullptr;
                destroy_vehicle(vit->get());
                vit = vehicles_.erase(vit);
            } else {
                ++vit;
            }
        }
        if (road->source) {
            auto& out = road->source->outgoing_edges;
            out.erase(std::remove(out.begin(), out.end(), road), out.end());
        }
        if (road->target) {
            auto& in = road->target->incoming_edges;
            in.erase(std::remove(in.begin(), in.end(), road), in.end());
        }
        graph_->edges.erase(it);
        removed = true;
    }
    return removed;
}
```

**Meaning:** `destroy_vehicle` detaches a vehicle from whatever road it's
standing on (if any) and from the Observer subject via `controller_.detach(v)`
— note the header comment's caveat that this does *not free* the vehicle;
the caller is responsible for erasing it from `vehicles_` right after, the
"same convention as `TrafficSimulator::destroy_vehicle_locked`" per the
header comment. `remove_road_locked` iterates both `id` and its reverse
twin `id + "_r"` (a two-way street's two `Road` objects, same pattern as
`load_road` above), and for each that actually exists: walks the *entire*
vehicle fleet with a manual iterator loop (not a range-for, because it
erases while iterating — `vit = vehicles_.erase(vit)` advances the iterator
to the next valid position, while the `else` branch manually increments),
destroying and erasing any vehicle currently on that road; then removes the
`Road*` pointer from its source/target intersections' adjacency lists
(`outgoing_edges`/`incoming_edges`) via the erase-remove idiom
(`std::remove` followed by `.erase`); then erases the edge itself from
`graph_->edges`, which (as the comment notes) triggers the owning
`unique_ptr`'s destructor to actually free the `Road`.

**Why:** the manual iterator loop (rather than `std::remove_if` +
`vehicles_.erase`) is necessary specifically because `destroy_vehicle(vit->get())`
is called for its side effect (detaching from the road and the controller)
*before* the container entry is removed — a `remove_if` predicate is
supposed to be a pure test, not something with side effects, so this can't
cleanly collapse into the idiom used just a few lines below for the
adjacency-list cleanup. The order of operations here matters for
correctness: vehicles must be handled *before* the `Road` object is erased
(since `destroy_vehicle` still needs `v->current_road` to be valid to call
`remove_vehicle` on it), and the adjacency-list cleanup must happen before
`graph_->edges.erase(it)` destroys the `Road`, since after that point
`road` is a dangling pointer.

**Usage:** `remove_road_locked` is the shared implementation behind the
public `remove_road` (a thin wrapper adding `reindex()`/`invalidate_routes()`)
and is also called directly from `remove_intersection` (for every road
touching the intersection being deleted) and from `split_road_near` (which
removes the original road before creating its two replacement halves).

**Check yourself:**
- Why can't `remove_road_locked`'s vehicle-destroying loop be rewritten as
  `vehicles_.erase(std::remove_if(...), vehicles_.end())`?
- What would go wrong if the adjacency-list cleanup (`outgoing_edges`/
  `incoming_edges`) ran *after* `graph_->edges.erase(it)` instead of before?

---

### `MapEditor.cpp` lines 77–83 and 203–224: `add_junction_at` and `add_road_between`

```cpp
std::string MapEditor::add_junction_at(double x, double y) {
    if (!graph_) return "";
    const std::string id = "j" + std::to_string(next_id_++);
    graph_->intersections[id] = std::make_unique<Intersection>(id, x, y);
    reindex();
    return id;
}

std::string MapEditor::add_road_between(const std::string& from, const std::string& to,
                                        double speed_kmh, int lanes, bool oneway) {
    if (!graph_) return "";
    auto fit = graph_->intersections.find(from);
    auto tit = graph_->intersections.find(to);
    if (fit == graph_->intersections.end() || tit == graph_->intersections.end()) return "";
    if (from == to) return "";

    if (lanes < 1) lanes = 1;
    if (speed_kmh < 5.0) speed_kmh = 5.0;
    const std::string id = "r" + std::to_string(next_id_++);
    graph_->create_directional_road(id, fit->second.get(), tit->second.get(), lanes, speed_kmh, oneway);
    if (!oneway) {
        graph_->create_directional_road(id + "_r", tit->second.get(), fit->second.get(),
                                        lanes, speed_kmh, oneway);
    }
    reindex();
    invalidate_routes();
    return id;
}
```

**Meaning:** `add_junction_at` generates a fresh id by prefixing the shared
`next_id_` counter with `"j"` (post-incrementing it so the next id caller —
of any kind — gets a distinct number, even across junctions, roads, and
buildings sharing the same counter), constructs an `Intersection` at the
given world coordinates, stores it, reindexes, and returns the new id.
`add_road_between` validates both endpoints exist and aren't the same node
(no self-loop roads), clamps `lanes` to at least 1 and `speed_kmh` to at
least 5 (sanity floors against a caller passing `0` or negative values from
a UI form), generates an `"r"`-prefixed id, and delegates the actual `Road`
construction to `graph_->create_directional_road` — note this is *not* the
`std::make_unique<Road>(...)` pattern `MapLoader.cpp` uses directly; here
the graph itself owns the construction helper (see
[`learn/02-core-graph-model.md`](02-core-graph-model.md) for what that
method does), a further sign of clean separation: `MapEditor` orchestrates
*when* and *whether* to add a road, while `TrafficGraph` owns *how* one gets
wired into the data structure.

**Why:** the shared `next_id_` counter (incremented regardless of which
kind of element is being created) guarantees every id across the *entire*
graph is globally unique, which is simpler and more robust than maintaining
separate per-kind counters that could theoretically be reset or diverge.
The input-clamping (`lanes < 1`, `speed_kmh < 5.0`) is defensive
programming against a UI that might not itself validate form input before
sending an edit request — the backend is the last line of defense against
a nonsensical `0`-lane road being created.

**Usage:** both are reachable from `TrafficSimulator`'s live map-editing
forwarders (see [`learn/07-simulation-core.md`](07-simulation-core.md)),
themselves driven by the frontend's map editor UI.

---

### `MapEditor.cpp` lines 85–201: `split_road_near` — the most involved method in this file

```cpp
std::string MapEditor::split_road_near(double x, double y) {
    if (!graph_) return "";

    Road* best = nullptr;
    double best_d = 1e18, best_t = 0.0;
    for (const auto& [rid, r] : graph_->edges) {
        if (!r->source || !r->target) continue;
        const double ax = r->source->x, ay = r->source->y;
        const double vx = r->target->x - ax, vy = r->target->y - ay;
        const double L2 = vx * vx + vy * vy;
        if (L2 < 1e-9) continue;
        double t = ((x - ax) * vx + (y - ay) * vy) / L2;
        if (t < 0.05) t = 0.05;
        if (t > 0.95) t = 0.95;
        const double d = std::hypot(ax + vx * t - x, ay + vy * t - y);
        if (d < best_d) { best_d = d; best = r.get(); best_t = t; }
    }
    if (!best) return "";
    ...
```

**Meaning:** this is the same perpendicular-projection math seen in
`MapLoader.cpp`'s `perp_distance` (compute `t`, the normalized position
along the segment closest to the click point, via a dot product divided by
squared segment length), but re-derived independently here rather than
reused — it clamps `t` to `[0.05, 0.95]` instead of `[0, 1]`, specifically
so a click near either end of a road can't produce a degenerate near-zero-
length split segment ("keep both halves long enough to drive," per the
inline comment). It linearly scans *every* road in the graph to find the
one geometrically closest to the click point `(x, y)`.

The rest of the method (lines 105–201) is the most intricate piece of logic
in this file: it locates the road's endpoints and properties, creates a new
`Intersection` at the interpolated split point, then — the hard part —
**preserves every vehicle currently on the road being split**. It records
each such vehicle's exact world position (`r->source->x + ux * v->position_on_road`,
not a fraction along the road) and unit heading `(dx, dy)` *before* deleting
the original road, because the inline comment explains precisely why
fractions won't work: the forward and reverse roads run in opposite
directions and the split point isn't necessarily the midpoint, so "0.5 along
the road" is ambiguous about which physical point it refers to once the
road is gone, while a world coordinate is not. After
`remove_road_locked(base)` deletes the original pair and two new
`id_a`/`id_b` road pairs are created (`create_directional_road`, mirrored
for the reverse direction if not one-way), each displaced vehicle is
re-homed onto whichever of the four possible new road objects
(`id_a`, `id_b`, and their `_r` twins) best matches its recorded position
and direction — filtering out any candidate road pointing the *wrong* way
(`ux * d.dx + uy * d.dy < 0.0`), then picking the closest match by combined
perpendicular-plus-clamping distance. A vehicle with no viable match at all
is destroyed rather than left in an inconsistent state.

**Why:** splitting a road while vehicles are actively driving on it is
exactly the kind of edit that could silently teleport traffic if done
carelessly (position it wrong by even a fraction of a road's length, and a
vehicle visibly jumps). The design explicitly optimizes for "a driver
crossing this point should not notice the edit at all" — the comment states
this goal directly. Capturing world coordinates and headings before
deletion, rather than trying to remap fractional positions after the fact,
is the only approach that's unambiguous given that a road and its split
halves don't share a consistent length-fraction coordinate system.

**Usage:** this is a live-editing feature reachable from
`TrafficSimulator`'s map API, letting a user insert a new junction into an
existing road (e.g. to add an intersection or a branch) while the
simulation keeps running, without needing to pause or reset it.

**Check yourself:**
- Why does `split_road_near` clamp its projection parameter `t` to
  `[0.05, 0.95]` while `MapLoader.cpp`'s `perp_distance` clamps the
  analogous parameter to `[0, 1]`? What different problem is each guarding
  against?
- Walk through why storing a displaced vehicle's position as world
  coordinates plus a heading (rather than "fraction along the original
  road") is the only unambiguous choice, once the two halves can be
  different lengths.
- What happens to a vehicle if `split_road_near` can't find any of the four
  candidate roads pointing the same way it was already heading?

---

### `MapEditor.cpp` lines 226–309: removal, updates, lights, and buildings

```cpp
bool MapEditor::update_road(const std::string& id, double speed_kmh, bool closed) {
    if (!graph_) return false;
    bool touched = false;
    for (const std::string& rid : {id, id + "_r"}) {
        auto it = graph_->edges.find(rid);
        if (it == graph_->edges.end()) continue;
        if (speed_kmh > 0.0) it->second->set_speed_limit(speed_kmh);
        it->second->set_closed(closed);
        it->second->update();
        touched = true;
    }
    if (touched) invalidate_routes();
    return touched;
}

bool MapEditor::set_light(const std::string& node_id, bool on, double green) {
    if (!graph_) return false;
    auto it = graph_->intersections.find(node_id);
    if (it == graph_->intersections.end()) return false;

    if (!on) {
        graph_->designalize(node_id);
        return true;
    }
    if (green < 5.0) green = 5.0;
    return graph_->signalize(node_id, green);
}
```

**Meaning:** `remove_road`/`remove_intersection` (lines 226–252) are thin
wrappers around `remove_road_locked` (`remove_intersection` additionally
collects every road touching the target node first, since a node can't be
deleted while roads still reference it as `source`/`target`). `update_road`
applies a new speed limit (only if positive — `0` or negative is treated as
"leave unchanged," not "set to zero") and open/closed state to *both*
directions of a two-way street uniformly, then calls `it->second->update()`
(presumably recomputing derived state like capacity or display congestion —
see [`learn/02-core-graph-model.md`](02-core-graph-model.md)) before
invalidating routes so vehicles replan around a newly-closed road.
`set_light` either tears down an existing signal (`designalize`) or installs
one (`signalize`, clamping `green` to a 5-second floor), delegating the
actual phase-group assignment logic to the graph rather than duplicating it
here — the comment notes `signalize()` itself refuses a junction with
"nothing to arbitrate" and reports that refusal back through its return
value, which `set_light` passes straight through to its own caller rather
than silently swallowing.

`add_building_at`/`remove_building` (lines 286–309) are simpler still:
build a small square footprint centered on the click point, generate a
unique key (auto-generated `"b17"`-style if unnamed, with a disambiguating
suffix loop if the requested name collides), and insert/erase from
`graph_->buildings` directly — no `reindex()`/`invalidate_routes()` calls,
consistent with buildings sitting outside the routable graph.

**Why:** applying `update_road` to both directions "so they stay
consistent" (per the inline comment) reflects a real invariant this
codebase maintains throughout: a two-way street's forward and reverse
`Road` objects are separate C++ objects but are meant to always represent
the same physical street, so any edit to one must apply to its twin or the
two would silently drift out of sync (a road closed in one direction but
still driveable in "reverse" makes no physical sense). `set_light`
deferring to `graph_->signalize`/`designalize` rather than manipulating
`light_system`/phase groups directly here is the same ownership discipline
seen in `add_road_between`'s use of `create_directional_road` — `MapEditor`
orchestrates *when* an edit happens; `TrafficGraph` owns *how* the graph's
internal invariants stay consistent while it happens.

**Usage:** every one of these methods is reachable from
`TrafficSimulator`'s public map-editing forwarders (see
[`learn/07-simulation-core.md`](07-simulation-core.md)), which is to say:
this entire file is the backend implementation of the frontend's "edit
mode" for the map.

**Check yourself:**
- Why does `update_road` treat `speed_kmh <= 0.0` as "don't change the
  speed" instead of, say, closing the road at speed zero?
- Why do `add_building_at`/`remove_building` skip the `reindex()`/
  `invalidate_routes()` calls that every road/junction mutator makes?

---

## `cpp_backend/include/simulation/HistoryStore.hpp` and `cpp_backend/src/simulation/HistoryStore.cpp`

### `HistoryStore.hpp` lines 1–22: the design rationale, stated up front

```cpp
#pragma once
// Persistent history of the simulated world within a single run, captured
// once per REAL second so a client can scrub back through the last
// kWindowSeconds of traffic without the live simulation ever pausing for
// anyone. Written to disk as it happens (not just held in RAM) so a crash
// mid-run doesn't lose everything captured before it -- but reset() wipes
// whatever a PRIOR run left behind: every launch of the simulator starts
// its own playback history from empty, it never resumes an old one.
//
// Indexed by REAL capture time ("seconds ago from now"), not sim_time_ --
// sim_time_ resets to 0 on every TrafficSimulator::load(), so two different
// real moments could otherwise both claim "t = 120s" and collide. ...
```

**Meaning:** this is `HistoryStore`'s purpose statement, and it answers the
syllabus's own prediction directly: yes, `HistoryStore` supports playback —
specifically time-scrubbing, letting a connected client rewind and view the
simulation as it looked at some earlier real moment, *without* the live
simulation itself pausing or being affected. Three design commitments are
stated explicitly: (1) capture cadence is once per real second, not once
per tick — decoupling history granularity from the tick rate (see
[`learn/07-simulation-core.md`](07-simulation-core.md)); (2) frames are
written to disk as captured, not buffered only in memory, so a crash
doesn't lose the whole session's history; (3) `reset()` deliberately
discards any *prior* run's history on startup — this is playback within one
session, not a permanent recording archive across restarts. The comment
also justifies indexing by **real** wall-clock time rather than the
simulation's own `sim_time_`: because `sim_time_` resets to `0` on every
`TrafficSimulator::load()` (map reload), two genuinely different moments
could otherwise both be labeled "t = 120s" and collide in the index — real
time never repeats within a session, so it's the only safe key.

**Why:** every one of these three design commitments directly answers a
concrete failure mode: memory-only history would be lost on any crash, an
unbounded and permanent history would grow forever and never let a fresh
session start clean, and indexing by `sim_time_` would actively produce
*wrong* answers (not just less convenient ones) across a map reload. This is
a case where the "why" for the whole file is written once, at the top,
rather than repeated per-function — a good pattern to recognize when
reading unfamiliar code: check the file-level comment before diving into
individual methods.

**Usage:** the entire rest of this file exists in service of this one
promise: `TrafficSimulator` calls `capture()` once per real second from
inside its tick loop (see `mtx_`-locking note below), and a query endpoint
calls `at()`/`range()` to serve scrub requests — this is the concrete
"HistoryStore ties to the tick loop" link the syllabus predicted.

---

### `HistoryStore.hpp` lines 23–102: the frame record types

```cpp
struct HistoryVehicleRecord {
    std::string id;
    std::string road;
    float pos = 0.0f;
    float speed = 0.0f;
    int lane = 0;
    float lateral_offset = 0.0f;
    int recalcs = 0;
    int state = 0;
    std::string type;
};

struct HistoryLightRecord {
    std::string id;
    std::string junction;
    int group = -1;
    float stop_from_end = 0.0f;
    unsigned int state = 0;
    bool has_head = false;
    float hx = 0.0f, hy = 0.0f, hux = 0.0f, huy = 0.0f;
};

struct HistoryRoadRecord {
    std::string id;
    float congestion = 0.0f;
    bool closed = false;
};

struct HistoryEventRecord { ... };
struct HistoryStatsRecord { ... };

struct HistoryFrame {
    double real_time = 0.0;
    double sim_time = 0.0;
    double calendar_time = 0.0;
    std::vector<HistoryVehicleRecord> vehicles;
    std::vector<HistoryLightRecord> lights;
    std::vector<HistoryRoadRecord> roads;
    std::vector<HistoryEventRecord> events;
    HistoryStatsRecord stats;
};
```

**Meaning:** five small aggregate structs, each a flattened, storage-shaped
mirror of one slice of live simulation state — vehicles, signal lights,
roads, events, and aggregate stats — followed by `HistoryFrame`, which
bundles one instant's worth of all five plus three time fields. Note the
deliberate use of `float` (not `double`) for most per-record numeric fields
(`pos`, `speed`, `lateral_offset`, `stop_from_end`, the head-position
fields) — half the storage of a `double`, acceptable because these values
are display-precision (pixel-scale positions and speeds), not something fed
back into physics. `HistoryLightRecord`'s comment explains a specific
design choice: it's keyed by the *approach road's* id, not the junction's,
"because a junction shows different colours to different approaches" — one
record per light *head*, not per intersection. `HistoryFrame`'s own comment
explains why it stores `roads` as "always the FULL current list" logically,
even though (as the `.cpp` reveals) the store internally decides whether to
persist that as a full baseline or a diff — callers of `capture()` never
need to think about deltas themselves.

**Why:** these types exist to be a bit-for-bit storage encoding target, not
a general-purpose live-simulation data structure — that's exactly why they
duplicate (in flattened, minimal-precision form) data that already exists
richly typed elsewhere (`Vehicle`, `Road`, `TrafficLight`; see
[`learn/02-core-graph-model.md`](02-core-graph-model.md),
[`learn/03-traffic-lights.md`](03-traffic-lights.md),
[`learn/05-vehicles.md`](05-vehicles.md)). Keeping a completely separate,
minimal record type (rather than serializing the live objects directly)
means the on-disk format's shape and precision are independent decisions
from the live simulation's own in-memory representation, and the disk
format can evolve (see `kVersion` in the `.cpp`, below) without touching
live-simulation types at all.

**Usage:** `TrafficSimulator::tick()` builds a `HistoryFrame` from its own
live state once per real second and passes it to `capture()`; `at()`
reconstructs and returns a `HistoryFrame` for a query. `to_state_json`
(below) converts a `HistoryFrame` into the same JSON shape
`TrafficSimulator::state_json()` produces live, so the frontend's existing
state-rendering code handles a historical frame identically to a live one.

---

### `HistoryStore.hpp` lines 104–179: the class contract and private layout

```cpp
// Not thread-safe on its own -- TrafficSimulator calls capture() from inside
// tick() with mtx_ already held, ...
class HistoryStore {
public:
    explicit HistoryStore(std::string dir, double window_seconds = 86400.0);
    void reset();
    void capture(const HistoryFrame& frame);

    struct Range {
        double oldest_ago = 0.0;
        double newest_ago = 0.0;
        double window_seconds = 0.0;
        bool empty = true;
    };
    Range range() const;
    std::optional<HistoryFrame> at(double seconds_ago) const;
    static std::string to_state_json(const HistoryFrame& frame);

private:
    struct IndexEntry { double real_time; std::size_t segment; uint64_t byte_offset; };
    struct Segment { std::string path; double start_real_time; double end_real_time; uint64_t baseline_offset; };

    void rotate_segment_locked(double real_time);
    void evict_locked(double now_real_time);
    std::optional<HistoryFrame> reconstruct_locked(const IndexEntry& target) const;

    std::string dir_;
    double window_seconds_;
    uint32_t session_id_;
    std::vector<Segment> segments_;
    std::vector<IndexEntry> index_;
    std::size_t next_segment_index_ = 0;
    std::vector<HistoryRoadRecord> last_roads_;
    double last_segment_start_ = -1.0;
};
```

**Meaning:** the default window is `86400.0` seconds — exactly 24 hours —
so out of the box a running simulator retains a full day of scrubbable
history before evicting the oldest frames. `explicit` on the constructor
prevents an accidental implicit conversion from a bare `std::string`
anywhere a `HistoryStore` is expected. Public methods split cleanly into
"lifecycle" (`reset`), "write" (`capture`), and "read" (`range`, `at`,
`to_state_json` — the last being `static` because it operates purely on a
`HistoryFrame` value, with no need for any `HistoryStore` instance state).
The `_locked` suffix on the three private helpers (`rotate_segment_locked`,
`evict_locked`, `reconstruct_locked`) is the exact same naming convention
`TrafficSimulator` itself uses (see
[`learn/07-simulation-core.md`](07-simulation-core.md)) to mark "this method
assumes a lock is already held by the caller" — here, per the class-level
comment, that's `TrafficSimulator`'s own `mtx_`, since `HistoryStore` (like
`MapEditor`) has no lock of its own. Private state layers three concerns:
identity/configuration (`dir_`, `window_seconds_`, `session_id_`), an
in-memory catalog of on-disk segment files plus a flat time-sorted index
into them (`segments_`, `index_`, `next_segment_index_`), and incremental
write state used only while actively capturing (`last_roads_`,
`last_segment_start_`).

**Why:** the `_locked` naming convention being reused identically here (and
in `MapEditor`) is a case of the codebase enforcing "consistent terminology"
at the level of a naming *pattern*, not just individual terms — once a
reader learns what a `_locked` suffix promises in `TrafficSimulator`, that
knowledge transfers immediately to any other class using the same
convention, without needing separate documentation each time. `session_id_`
existing at all (assigned once, at construction, from the current epoch
time — seen in the `.cpp`) is what disambiguates segment files across
*separate* process launches sharing the same `dir_`, even though `reset()`
deletes old segments before capture begins — a defense-in-depth measure in
case `reset()` somehow didn't run or another process is writing
concurrently.

**Usage:** `TrafficSimulator` constructs one `HistoryStore` instance as a
member (`history_{"data/history", 86400.0}`, confirmed by the grep at the
top of this module) and owns its entire lifecycle for the process's
duration.

**Check yourself:**
- Why is `to_state_json` a `static` method while every other public method
  on `HistoryStore` is an instance method?
- The comment says `HistoryStore` has "no lock of its own" — trace exactly
  which lock actually protects `capture()`/`at()`/`range()` from concurrent
  access, and where in `TrafficSimulator` that lock is held.
- Why keep a separate `session_id_` if `reset()` already deletes every
  leftover segment file from a prior run at startup?

---

### `HistoryStore.cpp` lines 13–51: the on-disk format constants and byte-buffer helpers

```cpp
constexpr uint32_t kMagic = 0x54484953;  // "THIS" (traffic HIStory), a sentinel
constexpr uint16_t kVersion = 2;  // 2: FrameHeader carries calendar_time
constexpr double kSegmentSpan = 3600.0;  // rotate into a new file every real hour

// Hand-rolled rather than pulling in a serialization library, matching the
// project's existing convention (VehicleSnapshotBin, BinarySnapshotHeader):
// this format has exactly one reader and one writer, both in this file.
void put_u8(std::vector<uint8_t>& b, uint8_t v) { b.push_back(v); }
void put_u16(std::vector<uint8_t>& b, uint16_t v) {
    b.push_back(static_cast<uint8_t>(v & 0xFF));
    b.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
}
...
```

**Meaning:** `kMagic` is a 4-byte sentinel value written at the start of
every segment file, spelling `"THIS"` when interpreted as ASCII bytes — a
common file-format trick that lets a reader immediately recognize (or
reject) a file as belonging to this format before trusting anything else
in it. `kVersion` is a small integer bumped whenever the binary layout
changes — the comment already documents that version `2` specifically added
`calendar_time` to `FrameHeader`, meaning at least one format migration has
already happened in this codebase's history. `kSegmentSpan = 3600.0` (one
hour) is the file-rotation interval, explained further below. The
`put_u8`/`put_u16`/`put_u32`/`put_i32`/`put_f32`/`put_f64`/`put_str` family
are minimal little-endian byte-buffer writers — each manually shifts and
masks a value's bytes into a `std::vector<uint8_t>` rather than using
`memcpy` for the integer cases (though `put_f32`/`put_f64` *do* use
`memcpy`+`memcpy`-equivalent bit-reinterpretation via a `bits` variable, to
get a `float`/`double`'s exact bit pattern without invoking undefined
behavior from a raw pointer-cast). `put_str` writes a 2-byte length prefix
(clamped to `0xFFFF`, i.e. a hard 65535-byte cap per string) followed by the
raw bytes.

**Why:** this is the same "no third-party serialization library, because
this format has exactly one reader and one writer, both in this file"
philosophy the comment for `Json.hpp` articulated for JSON — here applied to
a binary format instead, and the comment explicitly cross-references that
this matches an existing convention elsewhere in the codebase
(`VehicleSnapshotBin`, `BinarySnapshotHeader` — likely the WebSocket binary
state protocol covered in
[`learn/09-server-and-entrypoint.md`](09-server-and-entrypoint.md)).
Manual byte-shifting for integers (rather than a raw
`reinterpret_cast<uint8_t*>` + `memcpy`) guarantees a **fixed, portable
byte order** (little-endian, chosen explicitly) regardless of what native
endianness the host CPU uses — a real concern for a format meant to be
read back later, possibly on different hardware, whereas a raw memory dump
would silently bake in whatever endianness happened to be running at write
time.

**Usage:** every `HistoryFrame` field, transitively through
`write_frame_payload` (below), passes through one of these `put_*`
functions before hitting disk; the paired `Reader` struct (next) undoes
exactly this encoding.

---

### `HistoryStore.cpp` lines 53–111: the `Reader` struct and the two header layouts

```cpp
struct Reader {
    const uint8_t* p;
    size_t n;
    size_t pos = 0;
    uint8_t u8() { return p[pos++]; }
    uint16_t u16() { ... pos += 2; return v; }
    ...
    std::string str() {
        uint16_t len = u16();
        std::string s(reinterpret_cast<const char*>(p + pos), len);
        pos += len;
        return s;
    }
};

#pragma pack(push, 1)
struct SegmentFileHeader {
    uint32_t magic;
    uint16_t version;
    uint32_t session_id;
};
struct FrameHeader {
    double real_time;
    double sim_time;
    double calendar_time;
    uint32_t payload_bytes;
    uint16_t vehicle_count;
    uint16_t light_count;
    uint16_t road_count;
    uint16_t event_count;
    uint8_t is_baseline;
};
#pragma pack(pop)
```

**Meaning:** `Reader` is the mirror-image counterpart to the `put_*`
functions — a cursor (`pos`) over a raw byte buffer, with one method per
type that both decodes the value *and* advances `pos` past it, so callers
chain reads in the exact order the corresponding `put_*` calls wrote them,
without ever tracking offsets by hand. `#pragma pack(push, 1)` disables the
compiler's default struct field alignment/padding for the two header
structs that follow, meaning `SegmentFileHeader` is guaranteed to be
exactly `4 + 2 + 4 = 10` bytes on disk with no hidden padding bytes the
compiler might otherwise insert between fields for alignment — critical
because these structs are written and read via raw
`reinterpret_cast<char*>`/`sizeof(...)` (seen in `rotate_segment_locked`/
`capture` below), so any padding discrepancy between the writer's compiler
and a reader's compiler would silently corrupt the format. `SegmentFileHeader`
appears once, at the start of each segment file (`kMagic`, `kVersion`,
`session_id_`). `FrameHeader` appears once per captured frame, and notably
carries `payload_bytes` — the comment explains this exists "so a reader can
skip a frame without parsing it," i.e. a client scanning the file for a
specific timestamp doesn't need to fully decode every intervening frame's
vehicle/light/road/event records, just jump `payload_bytes` forward.
`is_baseline` (a single byte, effectively a bool) marks whether this frame's
road records are a full snapshot or a diff against the previous one (the
delta-encoding scheme covered in `capture`, below).

**Why:** `#pragma pack(push, 1)` paired with manual byte-order-controlled
`put_*` writers together form a coherent, defensive approach to a genuinely
fragile category of bug: binary formats that rely on "whatever the compiler
happens to do" for struct layout silently break the moment the compiler,
platform, or optimization settings change. This code takes both halves of
that risk explicitly under its own control instead. Storing
`payload_bytes` in every `FrameHeader` is a deliberate skip-ahead
optimization, directly enabling `HistoryStore::range()`/`at()`'s ability to
navigate a segment file without a full linear parse of everything in it.

**Usage:** `SegmentFileHeader` is written once per segment file by
`rotate_segment_locked` and could (though the code shown doesn't do so
explicitly) be checked on read for format compatibility via `kMagic`/
`kVersion`. `FrameHeader` is written once per `capture()` call and read once
per frame visited during `reconstruct_locked`'s linear scan.

**Check yourself:**
- Why does this code manually shift and mask bytes for integers instead of
  `reinterpret_cast`-ing a pointer and doing a raw `memcpy`, the way it does
  for `put_f32`/`put_f64`'s bit pattern?
- What could go wrong if `#pragma pack(push, 1)` were omitted from
  `FrameHeader`'s definition?
- Why does `FrameHeader` store `payload_bytes` explicitly instead of
  requiring a reader to parse `vehicle_count`/`light_count`/`road_count`/
  `event_count` to know how many bytes the payload occupies?

---

### `HistoryStore.cpp` lines 113–223: writing and reading a frame's payload

```cpp
void write_frame_payload(std::vector<uint8_t>& out, const HistoryFrame& f,
                          const std::vector<HistoryRoadRecord>& roads_to_write) {
    for (const auto& v : f.vehicles) { put_str(out, v.id); ... }
    for (const auto& l : f.lights) { ... }
    for (const auto& r : roads_to_write) { ... }
    for (const auto& e : f.events) { ... }
    put_f64(out, f.stats.avg_congestion);
    ...
}

struct ParsedFrame {
    std::vector<HistoryVehicleRecord> vehicles;
    std::vector<HistoryLightRecord> lights;
    std::vector<HistoryRoadRecord> roads;
    std::vector<HistoryEventRecord> events;
    HistoryStatsRecord stats;
};

ParsedFrame read_frame_payload(const std::vector<uint8_t>& buf, const FrameHeader& h) {
    Reader r{buf.data(), buf.size()};
    ParsedFrame out;
    out.vehicles.reserve(h.vehicle_count);
    for (uint16_t i = 0; i < h.vehicle_count; ++i) { ... }
    ...
    return out;
}
```

**Meaning:** `write_frame_payload` serializes each record type as a
flat, fixed-order sequence of fields, one record after another — vehicles,
then lights, then roads (using `roads_to_write`, a parameter distinct from
`f.roads` — this is the delta-vs-baseline decision point, made by the
*caller*, `capture()` below, not by this function), then events, then a
fixed block of five aggregate stats fields. `read_frame_payload` is the
exact structural mirror: it uses `FrameHeader`'s stored counts
(`vehicle_count`, `light_count`, etc.) to know how many of each record to
read, `reserve`-ing each output vector's capacity up front (avoiding
repeated reallocation as records are pushed), and returns a `ParsedFrame` —
a separate, purely-transient struct that holds "what one frame's payload
decoded to, *before* deltas are merged onto a running road map," per its
own doc comment. That distinction — `ParsedFrame` vs `HistoryFrame` — is
important: `ParsedFrame::roads` is exactly what one specific frame wrote to
disk (possibly just a delta), while a reconstructed `HistoryFrame::roads`
(built in `reconstruct_locked`, below) is the full merged road state at
that point in time.

**Why:** having a completely separate `ParsedFrame` type (rather than
reusing `HistoryFrame` for both "one frame's raw disk content" and "the
fully reconstructed instant") keeps the delta-merging logic
(`reconstruct_locked`) honest about a real distinction: a single stored
frame's `roads` list is *not* the same thing as "all roads worth mentioning
right now" once deltas are involved — conflating the two types would make
it easy to accidentally treat a delta as if it were complete state.

**Usage:** `write_frame_payload` is called once per `capture()`;
`read_frame_payload` is called once per frame visited while
`reconstruct_locked` walks a segment file forward from its baseline to the
target frame.

---

### `HistoryStore.cpp` lines 236–283: construction, `reset()`, and segment rotation

```cpp
HistoryStore::HistoryStore(std::string dir, double window_seconds)
    : dir_(std::move(dir)),
      window_seconds_(window_seconds),
      session_id_(static_cast<uint32_t>(now_epoch_seconds())) {
    std::error_code ec;
    fs::create_directories(dir_, ec);
}

void HistoryStore::reset() {
    segments_.clear();
    index_.clear();
    next_segment_index_ = 0;
    last_segment_start_ = -1.0;
    last_roads_.clear();

    std::error_code ec;
    if (!fs::exists(dir_, ec)) return;
    for (const auto& entry : fs::directory_iterator(dir_, ec)) {
        if (ec) break;
        if (entry.is_regular_file() && entry.path().extension() == ".bin") {
            std::error_code rm_ec;
            fs::remove(entry.path(), rm_ec);
        }
    }
}

void HistoryStore::rotate_segment_locked(double real_time) {
    Segment seg;
    seg.path = (fs::path(dir_) / ("hist-" + std::to_string(session_id_) + "-" +
                                   std::to_string(next_segment_index_) + ".bin")).string();
    ++next_segment_index_;
    seg.start_real_time = real_time;
    seg.end_real_time = real_time;

    SegmentFileHeader sh{kMagic, kVersion, session_id_};
    std::ofstream out(seg.path, std::ios::binary | std::ios::trunc);
    out.write(reinterpret_cast<const char*>(&sh), sizeof(sh));
    seg.baseline_offset = sizeof(sh);

    segments_.push_back(std::move(seg));
    last_segment_start_ = real_time;
    last_roads_.clear();
}
```

**Meaning:** the constructor stores `dir`, `window_seconds`, and derives
`session_id_` from the current epoch time truncated to a `uint32_t`, then
uses `std::filesystem::create_directories` (with an `std::error_code`
overload, meaning failures are reported via the code rather than an
exception) to ensure the target directory exists. `reset()` clears all
in-memory bookkeeping, then walks `dir_` deleting every `.bin` file found —
using error-code overloads of `fs::exists`/`directory_iterator`/`fs::remove`
throughout, so a filesystem hiccup degrades gracefully (skips a file, say)
rather than throwing and crashing the simulator's startup sequence.
`rotate_segment_locked` builds a filename embedding both `session_id_` and
`next_segment_index_` (guaranteeing uniqueness even across two processes
sharing a directory, or two segments within one run), writes a fresh
`SegmentFileHeader` at the very start of the new file (opened with
`std::ios::trunc`, discarding anything that might already exist at that
exact path), records `baseline_offset = sizeof(sh)` (the byte position
immediately after the header, where the first frame — the segment's
baseline — will be written), and resets `last_roads_` so the very next
`capture()` call knows to treat its frame as a fresh baseline rather than a
delta.

**Why:** using `std::error_code` overloads of every filesystem call
throughout `reset()` (rather than the exception-throwing overloads) is a
deliberate robustness choice specific to this being *startup* code — a
transient permission glitch or a file another process is mid-write on
shouldn't be able to prevent the whole simulator from launching just
because its playback history couldn't be perfectly cleaned. `last_roads_.clear()`
inside `rotate_segment_locked` is what makes each segment file
self-contained and independently reconstructable — the class comment on
`last_roads_` explicitly ties this to that goal: a query that needs to
reconstruct a frame late in an hour-long segment only ever needs to replay
*that* segment from its own baseline, never reach back into a previous
hour's file.

**Usage:** the constructor runs once, at `TrafficSimulator` startup, when
`history_{"data/history", 86400.0}` is member-initialized.
`rotate_segment_locked` runs from within `capture()` whenever the elapsed
time since the current segment started reaches `kSegmentSpan` (one hour) or
no segment exists yet.

---

### `HistoryStore.cpp` lines 285–349: `capture()` — delta encoding

```cpp
void HistoryStore::capture(const HistoryFrame& frame) {
    const bool need_new_segment =
        segments_.empty() || (frame.real_time - last_segment_start_) >= kSegmentSpan;
    if (need_new_segment) rotate_segment_locked(frame.real_time);

    Segment& seg = segments_.back();
    const bool is_baseline = (seg.start_real_time == frame.real_time);

    std::vector<HistoryRoadRecord> to_write;
    if (is_baseline) {
        to_write = frame.roads;
    } else {
        std::map<std::string, HistoryRoadRecord> current;
        for (const auto& r : frame.roads) current[r.id] = r;

        for (const auto& [id, rec] : current) {
            auto it = std::find_if(last_roads_.begin(), last_roads_.end(),
                                    [&](const HistoryRoadRecord& o) { return o.id == id; });
            const bool changed = it == last_roads_.end() ||
                                  it->congestion != rec.congestion || it->closed != rec.closed;
            if (changed) to_write.push_back(rec);
        }
        for (const auto& old : last_roads_) {
            if (!current.count(old.id)) {
                to_write.push_back(HistoryRoadRecord{old.id, 0.0f, false});
            }
        }
    }
    last_roads_ = frame.roads;
    ...
}
```

**Meaning:** every `capture()` call first decides whether the incoming
frame needs a *new* segment (no segment exists yet, or an hour has elapsed
since the current one's start), then determines `is_baseline` by simply
checking whether this frame's `real_time` exactly matches the segment's own
`start_real_time` — true only for the very first frame written into a fresh
segment. For a baseline frame, `to_write` is just the frame's entire
`roads` list, unmodified. For every subsequent frame in the same segment,
it builds a `current` map from the incoming frame's roads (keyed by id,
using `std::map` specifically so the following comparison and the later
"missing roads" pass are both naturally ordered/deterministic), then for
each road present in the current frame, looks it up by linear
`std::find_if` in `last_roads_` (the previous frame's road list) and only
adds it to `to_write` if it's new (`it == last_roads_.end()`) or its
`congestion`/`closed` fields actually changed — unchanged roads are simply
*not written*, saving space. A second loop then handles the inverse case:
a road present in `last_roads_` but missing from the current frame's
`roads` entirely means it fell back under `state_json()`'s own congestion
threshold and stopped being "worth mentioning" — but the comment explains
this still needs an explicit "back to normal" delta record
(`HistoryRoadRecord{old.id, 0.0f, false}`) written out, because in a delta
stream, *silence* means "unchanged," so simply omitting a road that quietly
became fine again would leave it frozen at its last-known congested value
forever during reconstruction.

**Why:** `HistoryFrame::roads` (per the header comment) already only ever
contains congested-or-closed roads — `state_json()`'s own filter, which
`HistoryStore` inherits rather than re-implements. Delta-encoding on top of
*that* already-filtered list is a second, independent compression layer:
most roads' congestion doesn't change from one real-second capture to the
next, so writing only the roads that changed (plus explicit "back to
normal" markers for roads that stopped needing mention) keeps typical
frame payloads small without losing any information needed to reconstruct
the full state later. The "silence means unchanged" delta convention is
exactly why the explicit reset-to-zero record is necessary — it's the one
case where *not* writing something would be actively wrong, not just less
thorough.

**Usage:** every real-second tick from `TrafficSimulator` (see
[`learn/07-simulation-core.md`](07-simulation-core.md)) funnels through
here; `reconstruct_locked` (below) is the function that walks this same
delta stream back into a full picture.

---

### `HistoryStore.cpp` lines 351–379: eviction and `range()`

```cpp
void HistoryStore::evict_locked(double now_real_time) {
    const double cutoff = now_real_time - window_seconds_;
    while (segments_.size() > 1 && segments_.front().end_real_time < cutoff) {
        std::error_code ec;
        fs::remove(segments_.front().path, ec);

        std::vector<IndexEntry> kept;
        kept.reserve(index_.size());
        for (const auto& e : index_) {
            if (e.segment == 0) continue;
            kept.push_back({e.real_time, e.segment - 1, e.byte_offset});
        }
        index_ = std::move(kept);
        segments_.erase(segments_.begin());
    }
}
```

**Meaning:** `evict_locked` deletes whole segment files (never individual
frames) from the front of `segments_` — oldest first — as long as more
than one segment remains (the currently-open segment, at
`segments_.size() == 1`, is never evicted, "so there is no gap in what's
retained" per the inline comment) and the evicted segment's *latest* frame
(`end_real_time`) is still older than the retention `cutoff`. Because
`IndexEntry::segment` stores a plain integer *index* into `segments_`
(rather than, say, a pointer or the segment's own id), removing
`segments_.front()` requires re-numbering every remaining index entry's
`segment` field down by one (`e.segment - 1`) — done here via a full rebuild
into a new `kept` vector, filtering out any index entry that pointed at the
now-deleted segment 0 (`if (e.segment == 0) continue;`) along the way.

**Why:** evicting by whole segment (not per-frame) is what keeps this
efficient — deleting a single hour-long `.bin` file is one filesystem call,
versus the alternative of scanning for and rewriting a file with only some
frames removed. The tradeoff is coarser granularity: history can only be
evicted an hour at a time, so retention is only *approximately*
`window_seconds_`, not exact to the second — acceptable given the stated
24-hour default window, where an hour of slack either way is immaterial.
Rebuilding `index_` on every eviction (rather than, say, storing segment
identity some way that survives reordering) is a straightforward tradeoff
favoring simplicity: evictions are infrequent (once per hour at most, given
the segment span), so paying an O(n) rebuild of the index each time is
cheap relative to how rarely it happens.

**`range()`** (lines 370–379) is much simpler: if `index_` is empty, return
the default (`empty = true`); otherwise report `newest_ago`/`oldest_ago` as
`now - <that entry's real_time>`, clamped to non-negative via `std::max`
(guarding against clock skew making a "future" timestamp look negative).

**Usage:** `evict_locked` runs at the end of every `capture()` call.
`range()` backs whatever endpoint tells the frontend's scrub UI how far
back it's allowed to seek (see
[`learn/09-server-and-entrypoint.md`](09-server-and-entrypoint.md)).

**Check yourself:**
- Why must at least one segment always remain un-evicted, even if its
  entire content is technically older than the retention window?
- Trace what happens to `index_` the moment `segments_.size()` drops from 2
  to 1 during `evict_locked` — which entries survive, and how do their
  `segment` fields change?

---

### `HistoryStore.cpp` lines 381–447: `reconstruct_locked` and `at()`

```cpp
std::optional<HistoryFrame> HistoryStore::reconstruct_locked(const IndexEntry& target) const {
    if (target.segment >= segments_.size()) return std::nullopt;
    const Segment& seg = segments_[target.segment];
    std::ifstream in(seg.path, std::ios::binary);
    if (!in) return std::nullopt;

    std::map<std::string, HistoryRoadRecord> roads;
    HistoryFrame result;
    result.real_time = target.real_time;
    bool found_target = false;

    uint64_t offset = seg.baseline_offset;
    while (true) {
        in.seekg(static_cast<std::streamoff>(offset));
        FrameHeader fh{};
        in.read(reinterpret_cast<char*>(&fh), sizeof(fh));
        if (!in) break;

        std::vector<uint8_t> payload(fh.payload_bytes);
        if (fh.payload_bytes > 0) {
            in.read(reinterpret_cast<char*>(payload.data()), ...);
            if (!in) break;
        }
        ParsedFrame parsed = read_frame_payload(payload, fh);

        for (const auto& r : parsed.roads) {
            if (r.congestion <= 0.0f && !r.closed) roads.erase(r.id);
            else roads[r.id] = r;
        }

        if (offset == target.byte_offset) {
            result.sim_time = fh.sim_time;
            ...
            found_target = true;
            break;
        }
        offset += sizeof(fh) + fh.payload_bytes;
    }
    if (!found_target) return std::nullopt;

    result.roads.reserve(roads.size());
    for (auto& [id, r] : roads) result.roads.push_back(std::move(r));
    return result;
}
```

**Meaning:** given an `IndexEntry` (a segment index plus the target frame's
exact byte offset within it), this walks the segment file **linearly from
its baseline**, frame by frame — not seeking directly to the target
offset — because reconstructing the target frame's *road* state requires
replaying every delta between the baseline and the target: it maintains a
running `roads` map, and for every frame visited (including frames before
the target), applies that frame's road deltas onto it — a record with
`congestion <= 0.0f && !closed` means "this road is back to normal," so it's
*erased* from the running map entirely (not stored as a zero-value entry;
the whole point of the original filter, inherited from `state_json()`, is
that "not worth mentioning" roads simply aren't present), while anything
else is inserted or overwritten. Once the loop reaches the exact byte
offset the target `IndexEntry` recorded, it captures that specific frame's
non-road fields (`sim_time`, `calendar_time`, `vehicles`, `lights`,
`events`, `stats` — all of which are complete per-frame, never delta-encoded,
unlike roads) and breaks. The final `result.roads` is built from whatever
survives in the running `roads` map at that point — the fully merged,
current-as-of-the-target-frame road state.

**`at()`** (lines 431–447) is the public entry point: it computes an
absolute `target_time` from `seconds_ago`, then `std::lower_bound`s into the
time-sorted `index_` to find the first entry not older than that target,
handling the two edge cases explicitly (before the earliest retained frame,
after the newest) by clamping to the nearest end, and otherwise comparing
the two neighboring candidates (`before`/`after`) to pick whichever is
closer in absolute time before finally calling `reconstruct_locked`.

**Why:** replaying from the baseline rather than seeking straight to the
target is the direct cost of the delta-encoding scheme in `capture()` — you
cannot know a road's current value at frame N without knowing whether any
frame between the baseline and N changed it, so a full linear scan through
the segment (up to one hour of frames, per `kSegmentSpan`) is the price paid
for capture()'s smaller per-frame writes. This is a real, deliberate
space-for-time tradeoff, not an oversight — writes (once per real second,
continuously, for the entire run) happen far more often than reads (only
when a client is actively scrubbing), so optimizing write size at some cost
to read latency is the right call for this access pattern. `lower_bound`'s
requirement that `index_` stay sorted by `real_time` is guaranteed by
construction — `capture()` only ever appends, and real time is monotonic —
so binary search is safe here without any extra bookkeeping.

**Usage:** `at()` is the sole read path this whole binary format exists to
serve — called from a query endpoint (see
[`learn/09-server-and-entrypoint.md`](09-server-and-entrypoint.md)) that
converts a user's scrub-bar position into a `seconds_ago` value.

**Check yourself:**
- Why can't `reconstruct_locked` simply `seekg` straight to
  `target.byte_offset` and parse only that one frame, given that the byte
  offset is already known?
- What does a `HistoryRoadRecord` with `congestion <= 0.0f && !closed`
  actually mean when it appears in a delta frame, and what does
  `reconstruct_locked` do with it that's different from an ordinary road
  record?
- Why is `at()`'s binary search over `index_` safe without any explicit
  "is this sorted" check?

---

### `HistoryStore.cpp` lines 449–525: `to_state_json`

```cpp
std::string HistoryStore::to_state_json(const HistoryFrame& frame) {
    std::ostringstream os;
    os << "{";
    os << "\"time\":" << frame.sim_time << ",";
    os << "\"calendarTime\":" << static_cast<long long>(frame.calendar_time) << ",";

    os << "\"vehicles\":[";
    bool first = true;
    for (const auto& v : frame.vehicles) {
        if (!first) os << ",";
        first = false;
        os << "{\"id\":" << jstr(v.id) << ",\"road\":" << jstr(v.road)
           << ",\"pos\":" << v.pos << ...
    }
    os << "],";
    ...
    os << "\"active\":" << frame.vehicles.size() << ",";
    os << "\"spawned\":" << frame.stats.spawned << ",";
    os << "\"stats\":{\"avgCongestion\":" << frame.stats.avg_congestion << ... << "}";
    os << "}";
    return os.str();
}
```

**Meaning:** this hand-builds a JSON object using `std::ostringstream`
concatenation — the same style `Json.hpp`'s design note favors for
high-frequency output, though notably `HistoryStore` defines its own tiny
private `jstr` helper (seen near the top of the `.cpp`, in the unnamed
namespace) rather than calling `json::escape`/`json::str` from `utils/Json.hpp` —
a small, self-contained duplication rather than a shared dependency. Each
array (`vehicles`, `lights`, `roads`, `events`) uses the same
`bool first = true; ... if (!first) os << ","; first = false;` pattern to
emit a properly comma-separated JSON array without a trailing comma. Field
names in the emitted JSON are deliberately abbreviated in places (`"n"` for
junction, `"g"` for group, `"d"` for stop-from-end, `"lat"` for
lateral_offset) — matching, per the inline comment, `state_json()`'s own
established wire format exactly, "so a query result can be handed to the
frontend's existing `applyState()` unchanged." `calendar_time` is explicitly
cast to `long long` before being streamed, with a comment cross-referencing
the identical concern in `TrafficSimulator::state_json()`: `ostringstream`'s
default precision is only 6 *significant* digits, which for an epoch-scale
value (~1.7 billion) would silently round to the nearest ~10,000 seconds if
streamed as a raw `double` — casting to an integer type sidesteps that
formatting default entirely. The final block explicitly calls out (in a
long inline comment) which live-`state_json()` fields are *deliberately
not* reproduced here — run-wide cumulative displays like the congestion
history chart or per-algorithm breakdown — because those describe the whole
run's history, not one instant, so during playback they should keep
showing their live current values rather than a historical snapshot.

**Why:** matching `state_json()`'s exact JSON shape field-for-field (down to
the same abbreviated key names) is the single most important design
decision in this function — it's what lets the frontend's existing
rendering code stay completely unaware of whether it's looking at live data
or a scrubbed historical frame, with zero special-casing needed on the
client. That's a strong form of interface discipline: rather than the
frontend needing an "if playback mode, parse this differently" branch,
`HistoryStore` does the work of conforming to the *existing* contract. The
explicit choice to *not* reproduce certain cumulative/run-wide fields is
equally deliberate, not an oversight — those fields are conceptually about
"the whole run so far," which doesn't change meaning as you scrub backward
in time, whereas everything else in this JSON genuinely represents "the
world at this one instant."

**Usage:** the query endpoint mentioned throughout this section (see
[`learn/09-server-and-entrypoint.md`](09-server-and-entrypoint.md)) calls
`at()` to get a `HistoryFrame`, then `to_state_json` to serialize it, and
returns that string as the HTTP response body directly.

**Check yourself:**
- Why does `to_state_json` define its own private `jstr` instead of reusing
  `json::str` from `utils/Json.hpp`? Is that a missed opportunity for code
  reuse, or does it make sense given each file's own stated design
  philosophy?
- Why is `frame.calendar_time` cast to `long long` before being streamed,
  while `frame.sim_time` is streamed as a raw `double` on the very line
  above it? What's different about the two values' typical magnitude?
- Name two fields `state_json()` emits live that `to_state_json` never
  reproduces, and explain in one sentence each why that's correct rather
  than an oversight.

---

## `cpp_backend/include/utils/StatisticsManager.hpp` and `cpp_backend/src/utils/StatisticsManager.cpp`

### `StatisticsManager.hpp` lines 1–20: purpose comment and travel metrics

```cpp
/*
 * Collects travel metrics, algorithm metrics and traffic metrics
 * (Statistics and Analysis Module of the project brief).
*/
#pragma once
#include <deque>
#include <map>
#include <string>

class StatisticsManager {
public:
    void recordTripCompleted(double travelTimeSeconds, double routeLengthMeters,
                             const std::string& vehicleType);
    int completedTrips() const;
    double averageTravelTime() const;
    double averageRouteLength() const;
    double totalTravelTime() const;
```

**Meaning:** the file-level comment states plainly that this class exists
to satisfy a named requirement — "Statistics and Analysis Module of the
project brief" — grouped into three metric families the comment lists up
front: travel, algorithm, and traffic metrics. `recordTripCompleted` is the
write side of the travel-metrics family: called once per vehicle that
finishes its trip, with the trip's total travel time, the route's total
length, and the vehicle's type (`"car"`, `"bus"`, etc. — see
[`learn/05-vehicles.md`](05-vehicles.md)). `completedTrips`/
`averageTravelTime`/`averageRouteLength`/`totalTravelTime` are read-only
accessors, all `const`, over derived/aggregated values.

**Why:** this class is explicitly a *metrics sink* — it accumulates numbers
fed to it by other systems as events happen (a trip completes, a route gets
computed), rather than computing anything itself from first principles. It
has no knowledge of `Vehicle`, `Road`, or `PathFinding` types at all (note
the header's include list has none of those) — it's deliberately decoupled
from the simulation's core types, only ever receiving plain
numbers/strings, which is what makes it trivially reusable/testable in
isolation.

**Usage:** `TrafficSimulator` calls `recordTripCompleted` whenever a vehicle
reaches its destination (see
[`learn/07-simulation-core.md`](07-simulation-core.md) for exactly where in
the tick loop a trip is recognized as "completed"), and exposes the
aggregate accessors (indirectly, via `toJson`, below) through its own
statistics-reporting API.

---

### `StatisticsManager.hpp` lines 22–41: algorithm metrics

```cpp
struct AlgorithmStats
{
    int computations = 0;
    int failures = 0;
    double totalMicros = 0.0;
    double totalNodesExplored = 0.0;
    double avgMicros() const {
        return computations ? totalMicros / computations : 0.0;
    }
    double avgNodes() const {
        return computations ? totalNodesExplored / computations : 0.0;
    }
};

void recordRouteComputation(const std::string& algorithm, double micros,
                            std::size_t nodesExplored, bool found);
const std::map<std::string, AlgorithmStats>& algorithmStats() const;
int totalRecalculations() const;
```

**Meaning:** `AlgorithmStats` is a small aggregate tracking, per named
routing algorithm, how many times it ran (`computations`), how many of
those runs failed to find a route (`failures`), and running totals of
microseconds spent and graph nodes explored — with two inline `const`
member functions computing derived averages on demand rather than storing
them (a guarded division: `computations ? ... : 0.0` avoids a
divide-by-zero before any computation has ever been recorded).
`recordRouteComputation` is the write side, called per route computation
with the algorithm's name as a free-form string key (rather than an enum),
the time it took, how many graph nodes it visited, and whether it
succeeded. `algorithmStats()` exposes the entire per-algorithm map by
`const&`. `totalRecalculations()` is a simpler global counter across every
algorithm combined.

**Why:** this is the concrete, code-level tie-in to the Strategy Pattern
covered in [`learn/04-routing.md`](04-routing.md):
`PathFinding`'s interchangeable concrete strategies (`BFS`, `Dijkstra`,
`AStar`) are exactly the "algorithm" this struct tracks per-name, letting
the simulator (or a developer) compare, e.g., `AStar`'s average
microseconds-per-computation against `Dijkstra`'s, or notice one algorithm
failing to find a route far more often than another — direct empirical
feedback on the Strategy choice, gathered automatically as the simulation
runs rather than through separate manual benchmarking (compare to
`route_bench`, the dedicated CMake benchmarking tool covered in
[`learn/01-orientation.md`](01-orientation.md), which serves a related but
distinct purpose — synthetic, repeatable microbenchmarks versus this
class's live, in-simulation telemetry). Keying by a string name rather than
an enum is what lets a new `PathFinding` strategy be added without touching
`StatisticsManager` at all — it just starts appearing as a new key in the
map the first time it's recorded.

**Usage:** every concrete `PathFinding` strategy's route computation (see
[`learn/04-routing.md`](04-routing.md))
is expected to report itself here via `recordRouteComputation`, likely
called from wherever `TrafficSimulator`/`Vehicle` invokes the routing
strategy (see [`learn/07-simulation-core.md`](07-simulation-core.md) and
[`learn/05-vehicles.md`](05-vehicles.md)).

---

### `StatisticsManager.hpp` lines 43–56: traffic metrics and JSON export

```cpp
struct Sample
{
    double simTime;
    double avgCongestion;
    int activeVehicles;
    double avgTravelTime;
};
void pushSample(double simTime, double avgCongestion, int activeVehicles);

const std::deque<Sample>& history() const;

std::string toJson(bool includeHistory = true) const;

private:
    int completedTrips_ = 0;
    double totalTravelTime_ = 0.0;
    double totalRouteLength_ = 0.0;
    std::map<std::string, int> tripsByType_;

    int routeComputations_ = 0;
    std::map<std::string, AlgorithmStats> algoStats_;

    std::deque<Sample> history_;
    static constexpr std::size_t kMaxSamples = 240;
```

**Meaning:** `Sample` is a point-in-time reading of aggregate traffic
health: the simulation time it was taken at, average congestion across the
network, how many vehicles were active, and (note) `avgTravelTime` is
*also* stored per-sample even though it's a cumulative-since-start value,
not something unique to that instant — this becomes relevant below.
`pushSample` appends a new sample, computing `avgTravelTime` itself
internally (calling the class's own `averageTravelTime()`) rather than
requiring the caller to pass it — so callers only ever need to supply the
genuinely instant-specific values. `history()` exposes the full sample deque
by `const&`. `toJson` is the single public serialization entry point, with
a `bool` parameter letting a caller opt out of including the (potentially
large) `history_` array in the output. Private state mirrors exactly the
three public metric families: trip totals plus a per-vehicle-type breakdown
map, algorithm computation counts plus the per-algorithm stats map, and a
bounded `std::deque<Sample>` capped at `kMaxSamples = 240`.

**Why:** `std::deque` (rather than `std::vector`) for `history_` is the
right container for a bounded FIFO that grows at the back and shrinks at
the front — `pop_front()` (seen in the `.cpp`, below) is O(1) on a `deque`
but O(n) on a `vector` (which would have to shift every remaining element
down). `kMaxSamples = 240` is a fixed cap enforced entirely inside
`pushSample` itself, meaning `StatisticsManager` self-limits its own memory
footprint regardless of how long the simulation has been running — this is
a deliberate, bounded-memory design distinct from `HistoryStore`'s
disk-backed, time-windowed retention: `StatisticsManager`'s history is a
small in-memory rolling window meant for a live congestion chart (240
points is enough for a reasonably dense line chart without needing
disk-backed persistence or scrubback), while `HistoryStore` is the
much heavier full-fidelity playback mechanism covered above. The two are
solving genuinely different problems despite both tracking "the simulation
over time."

**Usage:** `TrafficSimulator`'s tick loop calls `pushSample` periodically
(see [`learn/07-simulation-core.md`](07-simulation-core.md)) to build up
the in-memory congestion trend the frontend's live chart reads via
`toJson`'s `"history"` array — and per the earlier `HistoryStore.cpp`
walkthrough, `TrafficSimulator::state_json()` explicitly does *not*
reproduce this same "congestion history chart" data inside a historical
`HistoryStore` frame, precisely because it's meant to always show its live,
current value regardless of scrub position.

**Check yourself:**
- Why is `history_` a `std::deque` rather than a `std::vector`, given that
  both support random access and `push_back`?
- Why does `Sample` store `avgTravelTime` — a cumulative-since-start metric
  — inside what's otherwise a per-instant traffic snapshot? What would be
  lost if `Sample` only stored genuinely instantaneous fields?
- `AlgorithmStats` is keyed by a `std::string` algorithm name rather than an
  enum tied to the `PathFinding` Strategy hierarchy. What does that
  decouple, and what's the cost (if any) of using a string key instead?

---

### `StatisticsManager.cpp` lines 5–33: travel metrics implementation

```cpp
void StatisticsManager::recordTripCompleted(double travelTimeSeconds,
                                            double routeLengthMeters,
                                            const std::string& vehicleType)
{
    ++completedTrips_;
    totalTravelTime_ += travelTimeSeconds;
    totalRouteLength_ += routeLengthMeters;
    ++tripsByType_[vehicleType];
}

double StatisticsManager::averageTravelTime() const
{
    return completedTrips_ ? totalTravelTime_ / completedTrips_ : 0.0;
}
```

**Meaning:** `recordTripCompleted` is pure accumulation — increment a
counter, add to two running totals, and increment a per-type counter in
`tripsByType_` (`std::map<std::string, int>::operator[]` default-constructs
a fresh `0` entry the first time a new vehicle type appears, so no explicit
"does this type already exist" check is needed). `averageTravelTime` (and,
identically, `averageRouteLength`) compute a mean on demand from the
running totals, guarded against division by zero before any trip has
completed.

**Why:** storing running *totals* (`totalTravelTime_`, `totalRouteLength_`)
rather than every individual trip's raw values is the space-efficient
choice — this class never needs to recompute an average differently or
inspect individual historical trips, only ever the aggregate mean, so
O(1) space per metric (independent of how many trips have run) is
sufficient and strictly better than storing an ever-growing list. This is
a direct contrast with `HistoryStore`, which *does* need to retain
individual timestamped frames because its entire purpose is scrubbing back
to a specific past instant — `StatisticsManager` has no such requirement,
so it doesn't pay that storage cost.

**Usage:** called once per vehicle trip completion by `TrafficSimulator`
(see [`learn/07-simulation-core.md`](07-simulation-core.md)).

---

### `StatisticsManager.cpp` lines 35–56: algorithm metrics implementation

```cpp
void StatisticsManager::recordRouteComputation(const std::string& algorithm,
                                               double micros,
                                               std::size_t nodesExplored,
                                               bool found)
{
    ++routeComputations_;
    AlgorithmStats& s = algoStats_[algorithm];
    ++s.computations;
    if (!found) ++s.failures;
    s.totalMicros += micros;
    s.totalNodesExplored += static_cast<double>(nodesExplored);
}
```

**Meaning:** `algoStats_[algorithm]` looks up (or default-constructs, on
first use — `AlgorithmStats`'s member initializers make a fresh entry
harmless, all zeros) the stats struct for the named algorithm, then
increments its computation count, conditionally increments its failure
count, and adds this call's timing/node-exploration numbers to the running
totals. `routeComputations_` is a separate global tally across every
algorithm, kept alongside the per-algorithm breakdown so
`totalRecalculations()` doesn't need to sum the whole map every time it's
queried.

**Why:** `algoStats_[algorithm]` returning a reference (`AlgorithmStats&`)
rather than a copy means every subsequent `++`/`+=` on `s` mutates the
entry actually stored in the map directly — a small but important detail:
copying and reassigning would work too, but the reference is both simpler
to write and avoids an unnecessary struct copy per call. Maintaining
`routeComputations_` as a separately-tracked running total (rather than
deriving it from `algoStats_` on every `totalRecalculations()` call) is the
same "pay the cost at write time, not read time" tradeoff seen throughout
this module (compare `HistoryStore`'s baseline/delta scheme) — writes here
happen at most once per route computation, reads could happen far more
often (anytime the stats panel refreshes), so front-loading the sum avoids
repeated map traversal on the more frequent path.

**Usage:** called from wherever a `PathFinding` strategy actually runs (see
[`learn/04-routing.md`](04-routing.md)),
presumably by `Vehicle::request_route` or its caller in `TrafficSimulator`
(see [`learn/05-vehicles.md`](05-vehicles.md) and
[`learn/07-simulation-core.md`](07-simulation-core.md)) timing the call and
reporting the outcome immediately afterward.

---

### `StatisticsManager.cpp` lines 58–68: traffic metrics implementation — `pushSample`

```cpp
void StatisticsManager::pushSample(double simTime, double avgCongestion,
                                   int activeVehicles)
{
    history_.push_back({simTime, avgCongestion, activeVehicles, averageTravelTime()});
    while (history_.size() > kMaxSamples) history_.pop_front();
}
```

**Meaning:** constructs a new `Sample` via aggregate initialization
(`{simTime, avgCongestion, activeVehicles, averageTravelTime()}` — note the
fourth field, `avgTravelTime`, is *not* a parameter to `pushSample` at all;
it's computed fresh from this object's own running totals at the moment of
the call, answering the question raised in the header walkthrough above),
appends it to the back of `history_`, then evicts from the front in a
`while` loop until the deque's size is back within `kMaxSamples`.

**Why:** the `while` (rather than a single `if`) is defensive: it correctly
handles `kMaxSamples` being lowered at some point in the future, or any
other scenario where more than one sample needs evicting in a single call
— though in the current code, since exactly one sample is pushed per call
and the cap is a fixed `constexpr`, a single `if` would behave identically
today; the `while` is the more robust, assumption-free form. This is a
small but real code-quality observation: writing `while` here costs
nothing and removes a latent assumption (`kMaxSamples` never changes,
`push_back` never appends more than one item per call) that a future edit
could otherwise silently violate.

**Usage:** called periodically from `TrafficSimulator`'s tick loop (see
[`learn/07-simulation-core.md`](07-simulation-core.md)) to build the
in-memory congestion-trend chart data.

---

### `StatisticsManager.cpp` lines 70–115: `toJson`

```cpp
std::string StatisticsManager::toJson(bool includeHistory) const
{
    std::ostringstream os;
    os << "{";
    os << "\"completedTrips\":" << completedTrips_ << ",";
    os << "\"avgTravelTime\":" << json::num(averageTravelTime(), 2) << ",";
    os << "\"avgRouteLength\":" << json::num(averageRouteLength(), 2) << ",";
    os << "\"routeComputations\":" << routeComputations_ << ",";

    os << "\"tripsByType\":{";
    bool first = true;
    for (const auto& [type, count] : tripsByType_) {
        if (!first) os << ",";
        first = false;
        os << json::str(type) << ":" << count;
    }
    os << "},";

    os << "\"algorithms\":{";
    first = true;
    for (const auto& [name, s] : algoStats_) {
        ...
        os << json::str(name) << ":{"
           << "\"computations\":" << s.computations << ","
           << "\"failures\":" << s.failures << ","
           << "\"avgMicros\":" << json::num(s.avgMicros(), 2) << ","
           << "\"avgNodesExplored\":" << json::num(s.avgNodes(), 1) << "}";
    }
    os << "},";

    os << "\"history\":[";
    if (includeHistory) {
        first = true;
        for (const Sample& s : history_) {
            ...
            os << "{\"t\":" << json::num(s.simTime, 1)
               << ",\"congestion\":" << json::num(s.avgCongestion, 3)
               << ",\"vehicles\":" << s.activeVehicles
               << ",\"avgTravel\":" << json::num(s.avgTravelTime, 1) << "}";
        }
    }
    os << "]}";
    return os.str();
}
```

**Meaning:** this is the promised concrete usage of `utils/Json.hpp`'s
`json::num`/`json::str` helpers — every string value (`tripsByType_`'s
vehicle-type keys, `algoStats_`'s algorithm-name keys) goes through
`json::str` for correct quoting/escaping, and every floating-point value
goes through `json::num` with a call-site-chosen precision: 2 decimal
places for travel time and route length, 2 for microseconds, 1 for node
counts and sim time, 3 for congestion (the finest precision of the group,
appropriate for a value that's presumably a small fraction, e.g. `0.0`–`1.0`).
Integer fields (`completedTrips_`, `routeComputations_`, `s.computations`,
`s.failures`, `s.activeVehicles`) are streamed directly with `operator<<`,
no `json::num` needed since integers have no ambiguous decimal
representation to control. Both nested-object loops (`tripsByType_`,
`algoStats_`) and the array loop (`history_`) use the identical
`first`-flag comma-separation idiom seen in `HistoryStore::to_state_json`.
`includeHistory` gates only the innermost loop — the `"history":[` /`]`
wrapper is always emitted (as an empty array when `includeHistory` is
`false`), keeping the JSON's overall shape identical either way rather than
omitting the key entirely.

**Why:** always emitting the `"history"` key (just empty when suppressed)
rather than omitting it conditionally is a small but meaningful API
discipline choice — a client parsing this JSON never needs an
`if ("history" in response)` branch; the field is always present, only its
contents vary. The `includeHistory` flag itself exists as an obvious
bandwidth optimization: a caller that only needs the current aggregate
numbers (completed trips, averages) shouldn't have to pay for
re-transmitting up to 240 history samples on every request if it doesn't
need them.

**Usage:** this is the JSON blob that ultimately backs whatever "statistics
panel" the frontend renders (see
[`learn/09-server-and-entrypoint.md`](09-server-and-entrypoint.md) for the
exact HTTP/WebSocket surface), and per the earlier grep evidence, is
distinct from (though likely embedded alongside) the smaller `"stats"`
object `TrafficSimulator::state_json()` sends on every tick — the comment
found at `TrafficSimulator.cpp:713` ("in StatisticsManager instead, under
`"stats"`") signals that some subset of this same data is *also* summarized
inline in the live per-tick payload, while the fuller picture (including
the 240-sample history and full algorithm breakdown) is this dedicated
endpoint's job.

**Check yourself:**
- Why does `toJson` always emit the `"history"` key/array, just empty when
  `includeHistory` is `false`, instead of omitting the key entirely?
- Why are `s.computations`/`s.failures`/`completedTrips_` streamed with raw
  `operator<<` while `s.avgMicros()`/`averageTravelTime()` go through
  `json::num`? What type difference explains it?
- `avgCongestion` gets 3 decimal digits of precision while `avgTravelTime`
  gets 2 — propose a reason (from the values' likely ranges) for why the
  precisions differ.

---

## Summary

This module's five files sit at very different points on a
correctness-vs-performance spectrum, and each one's design tracks that
placement honestly rather than applying one style uniformly:

- **`Json.hpp`** hand-rolls a *writer* because it runs every tick; the
  vendored `nlohmann::json` handles *parsing* because that only runs at
  load time.
- **`MapLoader`** uses the vendored parser for exactly that reason, and
  spends most of its size on the OSM path — a large, heavily-commented
  effort to reconcile a format designed for maps and routing (OpenStreetMap)
  with one designed for a discrete-lane traffic simulator, including a
  documented, measured failure rate (92% of signals lost) that justified
  the fallback-walk algorithm actually shipped.
- **`MapEditor`** is a clean extraction from `TrafficSimulator` (references,
  not ownership; no lock of its own) whose hardest method,
  `split_road_near`, exists to make a live structural edit invisible to
  traffic already on the road being edited.
- **`HistoryStore`** trades read speed for write speed via baseline+delta
  segment files, indexed by real time (not `sim_time_`, which resets on
  reload) specifically to support scrubbing playback without pausing the
  live simulation.
- **`StatisticsManager`** is the simplest file in the module by design — a
  pure accumulator with no dependency on any core simulation type — and its
  `AlgorithmStats` map is the direct, empirical feedback loop on the
  `PathFinding` Strategy Pattern's real-world performance.

None of these files touch vehicle physics or the tick's per-frame
arbitration logic directly, but every one of them is reachable from
`TrafficSimulator` (see [`learn/07-simulation-core.md`](07-simulation-core.md))
and, transitively, from what the frontend renders — the next module,
[`learn/09-server-and-entrypoint.md`](09-server-and-entrypoint.md), covers
how that data actually reaches a browser.
