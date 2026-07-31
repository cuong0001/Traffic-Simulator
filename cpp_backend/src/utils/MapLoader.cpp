#include "utils/MapLoader.hpp"

#include "core/Building.hpp"
#include "core/Graph.hpp"
#include "core/Intersection.hpp"
#include "core/Road.hpp"
#include "core/TrafficLight.hpp"
#include "core/Zone.hpp"
#include "third_party/nlohmann/json.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {

using nlohmann::json;

// ---- Safe property readers (properties is a free-form key/value bag) --------

// String property; a number is accepted too (turned into a string), e.g. id: 7.
std::string str_prop(const json& props, const char* key, const std::string& fallback = "") {
    auto it = props.find(key);
    if (it == props.end()) return fallback;
    if (it->is_string()) return it->get<std::string>();
    if (it->is_number_integer()) return std::to_string(it->get<long long>());
    return fallback;
}

// Numeric property; a numeric string ("50") is accepted too.
double num_prop(const json& props, const char* key, double fallback) {
    auto it = props.find(key);
    if (it == props.end()) return fallback;
    if (it->is_number()) return it->get<double>();
    if (it->is_string()) {
        try {
            return std::stod(it->get<std::string>());
        } catch (...) {
            return fallback;
        }
    }
    return fallback;
}

bool bool_prop(const json& props, const char* key, bool fallback) {
    auto it = props.find(key);
    if (it == props.end() || !it->is_boolean()) return fallback;
    return it->get<bool>();
}

// Read a point [x, y] from a coordinates element; returns false if malformed.
bool read_point(const json& pt, double& x, double& y) {
    if (!pt.is_array() || pt.size() < 2 || !pt[0].is_number() || !pt[1].is_number()) {
        return false;
    }
    x = pt[0].get<double>();
    y = pt[1].get<double>();
    return true;
}

// ---- One handler per feature kind -------------------------------------------

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

    Intersection* node = new Intersection(id, x, y);
    node->display_name = str_prop(props, "name", id);
    node->zone = str_prop(props, "zone");
    node->is_roundabout = (node->zone == "roundabout");
    if (bool_prop(props, "signal", false)) {
        node->is_signalized = true;
        node->light_system = new TrafficLight(
            num_prop(props, "sgreen", 25.0), num_prop(props, "samber", 3.0),
            num_prop(props, "sred", 2.0),
            static_cast<int>(num_prop(props, "sgroups", 2)));
    }
    graph.intersections[id] = node;
}

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
        throw MapLoadError("road '" + id + "' references an unknown node ('" +
                           from + "' -> '" + to + "') in " + source);
    }
    const bool oneway = bool_prop(props, "oneway", false);
    if (graph.edges.count(id) || (!oneway && graph.edges.count(id + "_r"))) {
        throw MapLoadError("duplicate road id '" + id + "' in " + source);
    }

    const double length = num_prop(props, "length", 0.0);  // <= 0: measure geometrically
    const double speed = num_prop(props, "speed_kmh", 50.0);
    const int capacity = static_cast<int>(num_prop(props, "capacity", 12));
    const int lanes = static_cast<int>(num_prop(props, "lanes", 2));

    // The Road constructor wires the edge into both nodes' adjacency lists.
    graph.edges[id] = new Road(id, from_it->second, to_it->second, lanes, speed,
                               capacity, oneway, length);
    if (!oneway) {
        // Two-way street: expand into the reverse edge "id_r".
        graph.edges[id + "_r"] = new Road(id + "_r", to_it->second, from_it->second,
                                          lanes, speed, capacity, false, length);
    }
}

void load_zone(TrafficGraph& graph, const json& props, const json& geom) {
    if (geom.value("type", "") != "Polygon") return;  // background region: skip if malformed
    const json& coords = geom["coordinates"];
    if (!coords.is_array() || coords.empty() || !coords[0].is_array()) return;

    double min_x = 1e18, min_y = 1e18, max_x = -1e18, max_y = -1e18;
    bool has_point = false;
    for (const auto& pt : coords[0]) {
        double x = 0.0, y = 0.0;
        if (!read_point(pt, x, y)) continue;
        min_x = std::min(min_x, x);
        max_x = std::max(max_x, x);
        min_y = std::min(min_y, y);
        max_y = std::max(max_y, y);
        has_point = true;
    }
    if (!has_point) return;
    graph.zones.push_back({str_prop(props, "name"), str_prop(props, "kind"),
                           min_x, min_y, max_x - min_x, max_y - min_y});
}

void load_building(TrafficGraph& graph, const json& props, const json& geom) {
    if (geom.value("type", "") != "Polygon") return;
    const json& coords = geom["coordinates"];
    if (!coords.is_array() || coords.empty() || !coords[0].is_array()) return;

    std::vector<std::pair<double, double>> footprint;
    for (const auto& pt : coords[0]) {
        double x = 0.0, y = 0.0;
        if (read_point(pt, x, y)) footprint.emplace_back(x, y);
    }
    // GeoJSON repeats the first point at the end of a ring; drop that duplicate.
    if (footprint.size() >= 2 && footprint.front() == footprint.back()) {
        footprint.pop_back();
    }
    if (footprint.size() < 3) return;  // not enough points for a polygon

    const std::string name = str_prop(props, "name");
    const std::string kind = str_prop(props, "kind", "yes");
    const int levels = std::max(1, static_cast<int>(num_prop(props, "levels", 1)));

    // buildings is a map keyed by name: make a unique key when the name is empty
    // or already taken.
    std::string key = name.empty() ? "building" : name;
    if (graph.buildings.count(key)) {
        int i = 2;
        while (graph.buildings.count(key + " #" + std::to_string(i))) ++i;
        key += " #" + std::to_string(i);
    }
    Building* building = new Building(name.empty() ? key : name, kind, levels,
                                      std::move(footprint));
    // Project-specific extension: "node" anchors the building to a node so a
    // vehicle can pick it as a destination (Vehicle::request_route uses node_id).
    building->node_id = str_prop(props, "node");
    graph.buildings[key] = building;
}

// ===========================================================================
// OSM flavor: derive the graph from a raw OpenStreetMap / Overpass export.
// Ported from the reference project and adapted to this repo's TrafficGraph.
// ===========================================================================

// Local equirectangular projection constants (fine for city-scale extracts).
constexpr double kMetersPerDegLat = 110540.0;
constexpr double kMetersPerDegLon = 111320.0;
constexpr double kOriginMargin = 80.0;      // world margin around the map
constexpr double kSimplifyTolM = 3.0;       // Douglas-Peucker tolerance (meters)
constexpr double kMetersPerVehicle = 20.0;  // jam spacing -> capacity

struct Vec2 { double x = 0.0, y = 0.0; };

bool is_lonlat_range(double x, double y) {
    return x >= -180.0 && x <= 180.0 && y >= -90.0 && y <= 90.0;
}

// Drivable highway classes. Foot/cycle infrastructure is excluded.
bool is_drivable_highway(const std::string& hw) {
    static const std::unordered_set<std::string> kDrivable = {
        "motorway",       "trunk",         "primary",     "secondary",
        "tertiary",       "unclassified",  "residential", "living_street",
        "service",        "motorway_link", "trunk_link",  "primary_link",
        "secondary_link", "tertiary_link",
    };
    return kDrivable.count(hw) > 0;
}

double default_speed_kmh(const std::string& hw) {
    if (hw == "motorway") return 90.0;
    if (hw == "trunk") return 70.0;
    if (hw == "primary") return 60.0;
    if (hw == "secondary") return 50.0;
    if (hw == "tertiary" || hw == "unclassified") return 40.0;
    if (hw.size() > 5 && hw.compare(hw.size() - 5, 5, "_link") == 0) return 40.0;
    if (hw == "residential" || hw == "living_street") return 30.0;
    if (hw == "service") return 20.0;
    return 40.0;
}

int default_lanes_per_dir(const std::string& hw) {
    return (hw == "motorway" || hw == "trunk" || hw == "primary") ? 2 : 1;
}

int capacity_for(int lanes_per_dir, double length_m) {
    return std::max(2, static_cast<int>(
                           std::lround(lanes_per_dir * length_m / kMetersPerVehicle)));
}

// Perpendicular distance from p to segment a-b (for Douglas-Peucker).
double perp_distance(const Vec2& p, const Vec2& a, const Vec2& b) {
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double l2 = dx * dx + dy * dy;
    if (l2 <= 0.0) return std::hypot(p.x - a.x, p.y - a.y);
    double t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / l2;
    t = std::clamp(t, 0.0, 1.0);
    return std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

// Douglas-Peucker: marks which interior points of pts[a..b] to keep.
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

struct OsmWay {
    std::string id;                  // numeric part of "way/123", or an index
    std::vector<std::size_t> verts;  // indices into the vertex pool
    std::string name;
    std::string highway;
    double speed_kmh = 30.0;
    int lanes_per_dir = 1;
    bool oneway = false;             // geometry already oriented forward
};

// Add an OSM building (Polygon/MultiPolygon of lon/lat) to the graph, projecting
// each outer-ring point into world meters via `project`.
template <typename ProjectFn>
void load_building_osm(TrafficGraph& graph, const json& props, const json& geom,
                       ProjectFn&& project) {
    const std::string type = geom.value("type", "");
    const std::string name = str_prop(props, "name");
    const std::string kind = str_prop(props, "building", str_prop(props, "kind", "yes"));
    const int levels = std::max(1, static_cast<int>(num_prop(
                                        props, "building:levels", num_prop(props, "levels", 1))));

    auto ring_to_building = [&](const json& ring) {
        std::vector<std::pair<double, double>> footprint;
        for (const auto& pt : ring) {
            double x = 0.0, y = 0.0;
            if (!read_point(pt, x, y)) continue;
            const Vec2 v = project(x, y);
            footprint.emplace_back(v.x, v.y);
        }
        if (footprint.size() >= 2 && footprint.front() == footprint.back()) {
            footprint.pop_back();
        }
        if (footprint.size() < 3) return;

        std::string key = name.empty() ? "building" : name;
        if (graph.buildings.count(key)) {
            int i = 2;
            while (graph.buildings.count(key + " #" + std::to_string(i))) ++i;
            key += " #" + std::to_string(i);
        }
        graph.buildings[key] =
            new Building(name.empty() ? key : name, kind, levels, std::move(footprint));
    };

    if (type == "Polygon" && !geom["coordinates"].empty()) {
        ring_to_building(geom["coordinates"][0]);  // outer ring only
    } else if (type == "MultiPolygon") {
        for (const auto& poly : geom["coordinates"]) {
            if (!poly.empty()) ring_to_building(poly[0]);
        }
    }
}

void load_osm(TrafficGraph& graph, const std::vector<const json*>& road_features,
              const std::vector<const json*>& building_features,
              const std::vector<const json*>& signal_features,
              const std::string& source) {
    if (road_features.empty()) {
        throw MapLoadError("No drivable roads (highway=*) found in " + source);
    }

    // ---- vertex pool: exact coordinate -> index (OSM precision is 1e-7) ----
    std::map<std::pair<long long, long long>, std::size_t> vertex_index;
    std::vector<Vec2> raw_verts;  // lon/lat
    auto vertex_for = [&](double x, double y) {
        const auto key = std::make_pair(static_cast<long long>(std::llround(x * 1e7)),
                                        static_cast<long long>(std::llround(y * 1e7)));
        auto [it, inserted] = vertex_index.emplace(key, raw_verts.size());
        if (inserted) raw_verts.push_back({x, y});
        return it->second;
    };

    // ---- read ways --------------------------------------------------------
    std::vector<OsmWay> ways;
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
    if (ways.empty()) {
        throw MapLoadError("No usable road geometry in " + source);
    }

    // ---- traffic signals: keep only those that sit on a drivable vertex ----
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

    // ---- projection: lon/lat -> local meters, north up, shifted to a margin ----
    double min_x = 1e18, max_x = -1e18, min_y = 1e18, max_y = -1e18;
    for (const Vec2& v : raw_verts) {
        min_x = std::min(min_x, v.x); max_x = std::max(max_x, v.x);
        min_y = std::min(min_y, v.y); max_y = std::max(max_y, v.y);
    }
    const bool lonlat = is_lonlat_range(min_x, min_y) && is_lonlat_range(max_x, max_y);
    const double lon0 = (min_x + max_x) / 2.0, lat0 = (min_y + max_y) / 2.0;
    const double cos_lat0 = std::cos(lat0 * 3.14159265358979323846 / 180.0);
    auto project_base = [&](double x, double y) -> Vec2 {
        if (!lonlat) return {x, y};
        return {(x - lon0) * kMetersPerDegLon * cos_lat0,
                (lat0 - y) * kMetersPerDegLat};  // north stays up on screen
    };

    std::vector<Vec2> verts(raw_verts.size());
    Vec2 lo{1e18, 1e18};
    for (std::size_t i = 0; i < raw_verts.size(); ++i) {
        verts[i] = project_base(raw_verts[i].x, raw_verts[i].y);
        lo.x = std::min(lo.x, verts[i].x);
        lo.y = std::min(lo.y, verts[i].y);
    }
    const double shift_x = kOriginMargin - lo.x, shift_y = kOriginMargin - lo.y;
    for (Vec2& v : verts) { v.x += shift_x; v.y += shift_y; }
    auto project = [&](double x, double y) -> Vec2 {
        const Vec2 p = project_base(x, y);
        return {p.x + shift_x, p.y + shift_y};
    };

    // ---- junction detection: endpoints + vertices shared by >= 2 ways ----
    std::vector<int> use_count(verts.size(), 0);
    std::vector<bool> is_endpoint(verts.size(), false);
    std::map<std::size_t, std::set<std::string>> junction_names;
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

    // ---- lazy node creation ------------------------------------------------
    std::unordered_map<std::size_t, std::string> node_id_of;
    int node_counter = 0, waypoint_counter = 0;
    auto make_node = [&](std::size_t v, const std::string& id, const std::string& zone) {
        Intersection* node = new Intersection(id, verts[v].x, verts[v].y);
        node->zone = zone;
        node->is_roundabout = (zone == "roundabout");
        auto nit = junction_names.find(v);
        if (nit != junction_names.end() && !nit->second.empty()) {
            node->display_name = *nit->second.begin();
        }
        graph.intersections[id] = node;
        node_id_of.emplace(v, id);
        return id;
    };
    auto junction_node_id = [&](std::size_t v) {
        auto it = node_id_of.find(v);
        if (it != node_id_of.end()) return it->second;
        return make_node(v, "n" + std::to_string(node_counter++), "");
    };
    auto waypoint_node_id = [&](std::size_t v) {
        auto it = node_id_of.find(v);
        if (it != node_id_of.end()) return it->second;
        return make_node(v, "wp" + std::to_string(waypoint_counter++), "waypoint");
    };

    auto add_edge = [&](const std::string& id, const std::string& from,
                        const std::string& to, double len, double speed, int cap,
                        int lanes, bool oneway) {
        Intersection* a = graph.intersections[from];
        Intersection* b = graph.intersections[to];
        graph.edges[id] = new Road(id, a, b, lanes, speed, cap, oneway, len);
        if (!oneway) {
            graph.edges[id + "_r"] = new Road(id + "_r", b, a, lanes, speed, cap, false, len);
        }
    };

    // ---- split ways at junctions, simplify curves, emit edges --------------
    for (const OsmWay& w : ways) {
        int edge_counter = 0;
        std::size_t seg_start = 0;
        for (std::size_t i = 1; i < w.verts.size(); ++i) {
            if (!is_junction(w.verts[i]) && i + 1 < w.verts.size()) continue;

            // Segment w.verts[seg_start..i]: keep endpoints + significant bends.
            std::vector<Vec2> pts;
            pts.reserve(i - seg_start + 1);
            for (std::size_t k = seg_start; k <= i; ++k) pts.push_back(verts[w.verts[k]]);
            std::vector<bool> keep(pts.size(), false);
            keep.front() = keep.back() = true;
            simplify_keep(pts, 0, pts.size() - 1, kSimplifyTolM, keep);

            std::vector<std::size_t> chain;  // pool indices of kept vertices
            for (std::size_t k = 0; k < pts.size(); ++k) {
                if (keep[k]) chain.push_back(w.verts[seg_start + k]);
            }

            std::string prev_node = junction_node_id(chain.front());
            for (std::size_t k = 1; k < chain.size(); ++k) {
                const bool last = (k + 1 == chain.size());
                const std::string cur_node =
                    last ? junction_node_id(chain[k]) : waypoint_node_id(chain[k]);
                const Vec2& a = verts[chain[k - 1]];
                const Vec2& b = verts[chain[k]];
                const double len = std::hypot(b.x - a.x, b.y - a.y);
                if (len < 0.05 || prev_node == cur_node) {
                    prev_node = cur_node;
                    continue;  // degenerate duplicate point
                }
                std::string edge_id = "w" + w.id + "_" + std::to_string(edge_counter++);
                while (graph.edges.count(edge_id)) edge_id += "x";  // duplicate way in file
                add_edge(edge_id, prev_node, cur_node, len, w.speed_kmh,
                         capacity_for(w.lanes_per_dir, len), w.lanes_per_dir, w.oneway);
                prev_node = cur_node;
            }
            seg_start = i;
        }
    }

    // ---- keep only the largest strongly-connected component (Kosaraju) ----
    // Clipped OSM extracts leave one-way "traps"; inside one SCC every trip is
    // routable and disconnected fragments are dropped.
    std::unordered_map<std::string, std::vector<std::string>> fwd, rev;
    for (const auto& [id, road] : graph.edges) {
        fwd[road->source->name].push_back(road->target->name);
        rev[road->target->name].push_back(road->source->name);
    }
    std::vector<std::string> order;
    std::unordered_set<std::string> seen;
    for (const auto& [id, node] : graph.intersections) {
        if (seen.count(id)) continue;
        std::vector<std::pair<std::string, std::size_t>> stack{{id, 0}};
        seen.insert(id);
        while (!stack.empty()) {
            auto& [v, next] = stack.back();
            const std::vector<std::string>& nbrs = fwd[v];
            if (next < nbrs.size()) {
                const std::string w = nbrs[next++];
                if (seen.insert(w).second) stack.emplace_back(w, 0);
            } else {
                order.push_back(v);
                stack.pop_back();
            }
        }
    }
    std::unordered_set<std::string> assigned;
    std::vector<std::string> best_component;
    for (auto it = order.rbegin(); it != order.rend(); ++it) {
        if (assigned.count(*it)) continue;
        std::vector<std::string> component{*it};
        assigned.insert(*it);
        for (std::size_t head = 0; head < component.size(); ++head) {
            for (const std::string& w : rev[component[head]]) {
                if (assigned.insert(w).second) component.push_back(w);
            }
        }
        if (component.size() > best_component.size()) best_component = component;
    }
    std::unordered_set<std::string> keep_nodes(best_component.begin(),
                                               best_component.end());

    // Prune: delete edges touching a doomed node, then the doomed nodes, then
    // rebuild the adjacency lists from the survivors (avoids dangling pointers).
    std::vector<std::string> dead_edges;
    for (const auto& [id, road] : graph.edges) {
        if (!keep_nodes.count(road->source->name) ||
            !keep_nodes.count(road->target->name)) {
            dead_edges.push_back(id);
        }
    }
    for (const std::string& id : dead_edges) {
        delete graph.edges[id];
        graph.edges.erase(id);
    }
    std::vector<std::string> dead_nodes;
    for (const auto& [id, node] : graph.intersections) {
        if (!keep_nodes.count(id)) dead_nodes.push_back(id);
    }
    for (const std::string& id : dead_nodes) {
        delete graph.intersections[id];
        graph.intersections.erase(id);
    }
    for (auto& [id, node] : graph.intersections) {
        node->outgoing_edges.clear();
        node->incoming_edges.clear();
    }
    for (auto& [id, road] : graph.edges) {
        road->source->outgoing_edges.push_back(road);
        road->target->incoming_edges.push_back(road);
    }
    if (!dead_nodes.empty()) {
        std::fprintf(stderr,
                     "[map] pruned %zu node(s) outside the largest connected road "
                     "network (kept %zu)\n",
                     dead_nodes.size(), keep_nodes.size());
    }
    if (graph.intersections.empty()) {
        throw MapLoadError("Road network in " + source + " is empty after pruning");
    }

    // ---- signalize surviving junctions at/near a traffic_signals vertex ----
    int signalized = 0, skipped = 0;
    for (std::size_t v : signal_verts) {
        auto it = node_id_of.find(v);
        if (it == node_id_of.end()) continue;
        auto nit = graph.intersections.find(it->second);
        if (nit == graph.intersections.end()) continue;   // pruned away
        Intersection* node = nit->second;
        if (node->is_signalized) continue;
        // A traffic_signals point in OSM is a GUESS about this junction, so it
        // has to earn its light: signalize() clusters the approaches into two
        // opposing phase groups and refuses anything with nothing to
        // arbitrate. Mid-street signals in OSM are usually pedestrian
        // crossings, and stopping a straight street for itself is pure loss.
        if (graph.signalize(node->name)) ++signalized;
        else ++skipped;
    }
    if (signalized > 0) {
        std::fprintf(stderr,
                     "[map] signalized %d junction(s) from OSM traffic_signals"
                     " (%d skipped: nothing to arbitrate)\n",
                     signalized, skipped);
    }

    // ---- buildings (visual only) ------------------------------------------
    for (const json* f : building_features) {
        load_building_osm(graph, (*f)["properties"], (*f)["geometry"], project);
    }
}

}  // namespace

// ---------------------------------------------------------------------------

void MapLoader::load_from_geojson(TrafficGraph& graph, const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    // Use is_open(): on some toolchains (MinGW) operator! returns false for a
    // missing file until an actual read is attempted.
    if (!file.is_open()) {
        throw MapLoadError("cannot open map file: " + path);
    }
    std::ostringstream buffer;
    buffer << file.rdbuf();
    load_from_geojson_string(graph, buffer.str(), path);
}

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

    // Collect valid features and sort them. A single "uts:kind":"node" feature
    // means this is a native map; otherwise treat it as a raw OSM export and
    // derive the road graph from the geometry.
    std::vector<const json*> features;
    std::vector<const json*> osm_roads, osm_buildings, osm_signals;
    // write_geojson stamps this on everything it produces. Without it, a map
    // saved with no nodes in it has nothing to sniff and gets mistaken for an
    // OSM export, so the failure blames missing highway tags instead of saying
    // the graph was empty.
    bool native = root.value("generator", "") == "urban-traffic-simulator";
    for (const auto& f : root["features"]) {
        if (!f.is_object() || !f.contains("properties") || !f["properties"].is_object() ||
            !f.contains("geometry") || !f["geometry"].is_object() ||
            !f["geometry"].contains("coordinates")) {
            continue;
        }
        features.push_back(&f);
        const json& props = f["properties"];
        if (str_prop(props, "uts:kind") == "node") native = true;

        const std::string geom_type = f["geometry"].value("type", "");
        if (geom_type == "LineString" && is_drivable_highway(str_prop(props, "highway"))) {
            osm_roads.push_back(&f);
        } else if ((geom_type == "Polygon" || geom_type == "MultiPolygon") &&
                   props.contains("building")) {
            osm_buildings.push_back(&f);
        } else if (geom_type == "Point" &&
                   str_prop(props, "highway") == "traffic_signals") {
            osm_signals.push_back(&f);
        }
    }

    if (!native) {
        load_osm(graph, osm_roads, osm_buildings, osm_signals, source_name);
        return;
    }

    // ---- native flavor: two passes over the uts:kind features --------------
    // Pass 1: nodes first, so pass 2 can resolve each road's from/to.
    for (const json* f : features) {
        const json& props = (*f)["properties"];
        if (str_prop(props, "uts:kind") == "node") {
            load_node(graph, props, (*f)["geometry"], source_name);
        }
    }
    if (graph.intersections.empty()) {
        throw MapLoadError(source_name + " contains no nodes (empty graph)");
    }

    // Pass 2: roads, zones, buildings.
    for (const json* f : features) {
        const json& props = (*f)["properties"];
        const std::string kind = str_prop(props, "uts:kind");
        if (kind == "road") {
            load_road(graph, props, source_name);
        } else if (kind == "zone") {
            load_zone(graph, props, (*f)["geometry"]);
        } else if (kind == "building") {
            load_building(graph, props, (*f)["geometry"]);
        }
    }

    // Pass 3: phase groups. A node carries its "signal" flag from pass 1, but a
    // group is a property of the APPROACHES, which only exist now.
    //
    // Accept a single group here. The map file states this signal explicitly,
    // so dropping it would mean save-then-load quietly loses a light the author
    // placed -- the round trip has to be lossless. Only a node that cannot
    // carry a signal at all (a roundabout, a waypoint, no approaches) gives it
    // up, because leaving its approaches unassigned reads as "no group", which
    // fails closed and would hold the junction on red forever.
    for (auto& [id, node] : graph.intersections) {
        if (!node->is_signalized) continue;
        if (graph.assign_signal_groups(id) > 0) {
            if (node->light_system) node->light_system->set_phase_offset_from(id);
        } else {
            graph.designalize(id);
        }
    }
}

std::string MapLoader::validate(const TrafficGraph& graph) {
    if (graph.intersections.empty()) return "graph is empty";

    // Single BFS from an arbitrary node following outgoing edges.
    const std::string origin = graph.intersections.begin()->first;
    std::unordered_set<std::string> reached{origin};
    std::vector<const Intersection*> frontier{graph.intersections.begin()->second};
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

std::string MapLoader::map_json_string(const TrafficGraph& graph) {
    json out;
    out["intersections"] = json::array();
    out["roads"] = json::array();
    out["buildings"] = json::array();
    out["zones"] = json::array();

    for (const auto& [id, node] : graph.intersections) {
        out["intersections"].push_back({
            {"id", id},
            {"x", node->x},
            {"y", node->y},
            {"zone", node->zone},
            {"signal", node->is_signalized},
        });
    }

    for (const auto& [id, road] : graph.edges) {
        // A two-way street is stored as "id" + "id_r"; emit it once by skipping
        // the reverse twin when its base direction is present.
        if (id.size() > 2 && id.compare(id.size() - 2, 2, "_r") == 0 &&
            graph.edges.count(id.substr(0, id.size() - 2))) {
            continue;
        }
        out["roads"].push_back({
            {"id", id},
            {"from", road->source->name},
            {"to", road->target->name},
            {"length", road->length},
            {"lanes", road->lanes},
            {"speed_kmh", road->speed_limit},
            {"oneway", road->is_one_way},
        });
    }

    for (const auto& [name, building] : graph.buildings) {
        json footprint = json::array();
        for (const auto& [px, py] : building->footprint) {
            footprint.push_back({px, py});
        }
        out["buildings"].push_back({
            {"name", building->name},
            {"kind", building->type},
            {"levels", building->levels},
            {"footprint", std::move(footprint)},
        });
    }

    for (const Zone& z : graph.zones) {
        out["zones"].push_back({
            {"name", z.name},
            {"kind", z.kind},
            {"x", z.x},
            {"y", z.y},
            {"w", z.w},
            {"h", z.h},
        });
    }

    return out.dump();  // compact: city-scale maps are big, skip pretty-printing
}

void MapLoader::write_map_json(const TrafficGraph& graph, const std::string& path) {
    const std::string text = map_json_string(graph);
    std::ofstream file(path, std::ios::binary);
    if (!file.is_open()) {
        throw MapLoadError("cannot write map JSON: " + path);
    }
    file << text;
    if (!file) {
        throw MapLoadError("failed while writing map JSON: " + path);
    }
}

void MapLoader::write_geojson(const TrafficGraph& graph, const std::string& path) {
    json features = json::array();

    // --- nodes -------------------------------------------------------------
    for (const auto& [id, node] : graph.intersections) {
        json props{
            {"uts:kind", "node"},
            {"id", id},
            {"name", node->display_name},
            {"zone", node->zone},
        };
        if (node->is_signalized && node->light_system) {
            props["signal"] = true;
            props["sgroups"] = node->light_system->get_groups();
            props["sgreen"] = node->light_system->get_green_duration();
            props["samber"] = node->light_system->get_yellow_duration();
            props["sred"] = node->light_system->get_red_duration();
        }
        features.push_back({
            {"type", "Feature"},
            {"properties", std::move(props)},
            {"geometry", {{"type", "Point"}, {"coordinates", {node->x, node->y}}}},
        });
    }

    // --- roads (a two-way street is one feature; skip the "_r" twin) -------
    for (const auto& [id, road] : graph.edges) {
        if (id.size() > 2 && id.compare(id.size() - 2, 2, "_r") == 0 &&
            graph.edges.count(id.substr(0, id.size() - 2))) {
            continue;
        }
        features.push_back({
            {"type", "Feature"},
            {"properties", {
                {"uts:kind", "road"},
                {"id", id},
                {"from", road->source->name},
                {"to", road->target->name},
                {"length", road->length},
                {"speed_kmh", road->speed_limit},
                {"capacity", road->capacity},
                {"lanes", road->lanes},
                {"oneway", road->is_one_way},
            }},
            {"geometry", {{"type", "LineString"}, {"coordinates", json::array({
                json::array({road->source->x, road->source->y}),
                json::array({road->target->x, road->target->y}),
            })}}},
        });
    }

    // --- buildings ---------------------------------------------------------
    for (const auto& [key, building] : graph.buildings) {
        if (building->footprint.empty()) continue;   // nothing to draw or save
        json ring = json::array();
        for (const auto& [px, py] : building->footprint) ring.push_back({px, py});
        ring.push_back({building->footprint.front().first,
                        building->footprint.front().second});  // GeoJSON rings close
        json props{
            {"uts:kind", "building"},
            {"name", building->name},
            {"kind", building->type},
            {"levels", building->levels},
        };
        if (!building->node_id.empty()) props["node"] = building->node_id;
        features.push_back({
            {"type", "Feature"},
            {"properties", std::move(props)},
            {"geometry", {{"type", "Polygon"}, {"coordinates", json::array({std::move(ring)})}}},
        });
    }

    // --- zones (display-only rectangles) -----------------------------------
    for (const Zone& z : graph.zones) {
        json ring = json::array({
            json::array({z.x, z.y}),
            json::array({z.x + z.w, z.y}),
            json::array({z.x + z.w, z.y + z.h}),
            json::array({z.x, z.y + z.h}),
            json::array({z.x, z.y}),
        });
        features.push_back({
            {"type", "Feature"},
            {"properties", {{"uts:kind", "zone"}, {"name", z.name}, {"kind", z.kind}}},
            {"geometry", {{"type", "Polygon"}, {"coordinates", json::array({std::move(ring)})}}},
        });
    }

    const json out{
        {"type", "FeatureCollection"},
        {"generator", "urban-traffic-simulator"},
        {"features", std::move(features)},
    };

    std::ofstream file(path, std::ios::binary);
    if (!file.is_open()) {
        throw MapLoadError("cannot write GeoJSON: " + path);
    }
    file << out.dump();
    if (!file) {
        throw MapLoadError("failed while writing GeoJSON: " + path);
    }
}
