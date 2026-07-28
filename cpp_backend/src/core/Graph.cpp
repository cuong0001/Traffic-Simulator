#include "core/Graph.hpp"
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
