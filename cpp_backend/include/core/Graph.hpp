#pragma once
#include <string>
#include <unordered_map>
#include <vector>
#include "core/Zone.hpp"

// Use forward declarations instead of #include to speed up builds and avoid circular dependencies
class Intersection;
class Road;
class Building;

class TrafficGraph {
public:
    // Store objects in unordered_maps keyed by name/ID -> object pointer
    std::unordered_map<std::string, Intersection*> intersections;
    std::unordered_map<std::string, Road*> edges;
    std::unordered_map<std::string, Building*> buildings;
    std::vector<Zone> zones; // Display-only background regions, loaded from the GeoJSON map

    // Construct an empty graph, then populate it with MapLoader
    TrafficGraph() = default;
    // Build the graph from CSV file paths
    TrafficGraph(const std::string& node_path, const std::string& road_path, const std::string& building_path);
    // Destroy the graph, freeing all Intersection, Road, and Building memory
    ~TrafficGraph();

    // Data loading and graph construction
    void load_from_csv(const std::string& node_path, const std::string& road_path, const std::string& building_path);
    void create_directional_road(const std::string& edge_id, Intersection* source_node, Intersection* target_node, int lanes, double speed_limit, bool is_one_way);

    // Lookup
    Road* get_edge(const std::string& edge_name) const;
}; 
