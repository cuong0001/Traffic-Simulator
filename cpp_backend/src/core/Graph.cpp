#include "core/Graph.hpp"

#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>
#include "core/Intersection.hpp"
#include "core/Road.hpp"
#include "core/Building.hpp"

// Destroy the graph, freeing all memory
TrafficGraph::~TrafficGraph() {
    for (auto& pair : intersections) {
        delete pair.second;
    }
    intersections.clear();

    for (auto& pair : edges) {
        delete pair.second;
    }
    edges.clear();

    for (auto& pair : buildings) {
        delete pair.second;
    }
    buildings.clear();
}

// Build a road (directed edge)
void TrafficGraph::create_directional_road(const std::string& edge_id, Intersection* source_node, Intersection* target_node, 
                                           int lanes, double speed_limit, bool is_one_way) {
    Road* new_road = new Road(edge_id, source_node, target_node, lanes, speed_limit);
    new_road->is_one_way = is_one_way;
    edges[edge_id] = new_road;
}

// Look up a Road*
Road* TrafficGraph::get_edge(const std::string& edge_name) const {
    auto it = edges.find(edge_name);
    if (it != edges.end()) {
        return it->second;
    }
    return nullptr;
}

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
        std::string base_id = id.size() > 2 && id.compare(id.size() - 2, 2, "_r") == 0
                                  ? id.substr(0, id.size() - 2)
                                  : id;
        auto [it, inserted] = base_name_to_id.try_emplace(base_id, base_idx);
        if (inserted) base_idx++;
        road->base_index = it->second;
    }
}

// ---------------------------------------------------------------------------
// Signals
// ---------------------------------------------------------------------------
int TrafficGraph::assign_signal_groups(const std::string& node_id) {
    auto it = intersections.find(node_id);
    if (it == intersections.end()) return 0;
    Intersection* node = it->second;
    if (node->is_roundabout || node->zone == "waypoint") return 0;

    const std::vector<Road*>& in = node->incoming_edges;
    if (in.empty()) return 0;

    // Approach bearing = the direction a vehicle is travelling as it reaches
    // the node. Two approaches on the same street arrive on the same AXIS
    // (bearings 180 degrees apart), which is why the comparison folds by pi:
    // north-bound and south-bound traffic on one street shares a group and
    // goes green together, while the cross street waits.
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

    // One axis only: put everyone in group 0 rather than leaving half the
    // approaches unassigned. The junction then just cycles for all of them,
    // which is the honest answer when there is no crossing stream to hold.
    const bool crossing = has0 && has1;
    for (std::size_t i = 0; i < in.size(); ++i) {
        in[i]->signal_group = crossing ? group[i] : 0;
        // Keep the stop line inside the road. On a short OSM block a fixed 8 m
        // would sit before the road even starts, and vehicles would brake for
        // a line behind them.
        in[i]->signal_stop_from_end = std::min(8.0, in[i]->length * 0.4);
    }
    return crossing ? 2 : 1;
}

bool TrafficGraph::signalize(const std::string& node_id, double green) {
    // A guessed signal has to earn its place: only a real crossing gets one.
    if (assign_signal_groups(node_id) < 2) return false;
    Intersection* node = intersections[node_id];
    delete node->light_system;
    node->light_system = new TrafficLight(green, 3.0, 2.0, 2);
    node->light_system->set_phase_offset_from(node->name);
    node->is_signalized = true;
    return true;
}

void TrafficGraph::designalize(const std::string& node_id) {
    auto it = intersections.find(node_id);
    if (it == intersections.end()) return;
    Intersection* node = it->second;
    for (Road* r : node->incoming_edges) r->signal_group = -1;
    delete node->light_system;
    node->light_system = nullptr;
    node->is_signalized = false;
}
