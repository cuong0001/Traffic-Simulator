#include "../../include/MapLoader.h"

#include "../../include/Building.h"
#include "../../include/Graph.h"
#include "../../include/Node.h"
#include "../../include/Road.h"
#include "../../include/TrafficLight.h"
#include "../../include/Zone.h"
#include "../../include/third_party/nlohmann/json.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>
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
    if (graph.nodes.count(id)) {
        throw MapLoadError("duplicate node id '" + id + "' in " + source);
    }
    double x = 0.0, y = 0.0;
    if (!read_point(geom["coordinates"], x, y)) {
        throw MapLoadError("node '" + id + "' has malformed coordinates in " + source);
    }

    Node* node = new Node(id, x, y);
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
    graph.nodes[id] = node;
}

void load_road(TrafficGraph& graph, const json& props, const std::string& source) {
    const std::string id = str_prop(props, "id");
    const std::string from = str_prop(props, "from");
    const std::string to = str_prop(props, "to");
    if (id.empty() || from.empty() || to.empty()) {
        throw MapLoadError("road feature is missing id/from/to in " + source);
    }
    auto from_it = graph.nodes.find(from);
    auto to_it = graph.nodes.find(to);
    if (from_it == graph.nodes.end() || to_it == graph.nodes.end()) {
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

    // Collect the valid features (those with properties + geometry.coordinates).
    std::vector<const json*> features;
    for (const auto& f : root["features"]) {
        if (f.is_object() && f.contains("properties") && f["properties"].is_object() &&
            f.contains("geometry") && f["geometry"].is_object() &&
            f["geometry"].contains("coordinates")) {
            features.push_back(&f);
        }
    }

    // Pass 1: nodes first, so pass 2 can resolve each road's from/to.
    for (const json* f : features) {
        const json& props = (*f)["properties"];
        if (str_prop(props, "uts:kind") == "node") {
            load_node(graph, props, (*f)["geometry"], source_name);
        }
    }
    if (graph.nodes.empty()) {
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
}

std::string MapLoader::validate(const TrafficGraph& graph) {
    if (graph.nodes.empty()) return "graph is empty";

    // Single BFS from an arbitrary node following outgoing edges.
    const std::string origin = graph.nodes.begin()->first;
    std::unordered_set<std::string> reached{origin};
    std::vector<const Node*> frontier{graph.nodes.begin()->second};
    for (std::size_t head = 0; head < frontier.size(); ++head) {
        for (const Road* road : frontier[head]->outgoing_edges) {
            if (reached.insert(road->target->name).second) {
                frontier.push_back(road->target);
            }
        }
    }
    const std::size_t unreachable = graph.nodes.size() - reached.size();
    if (unreachable > 0) {
        return std::to_string(unreachable) + " node(s) unreachable from '" + origin + "'";
    }
    return "";
}
