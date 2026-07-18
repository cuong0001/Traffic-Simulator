#include "core/Graph.hpp"
#include "core/Intersection.hpp"
#include "core/Road.hpp"
#include "core/Building.hpp"
#include <fstream>
#include <sstream>
#include <iostream>

// Legacy constructor (previously JSON-based); now loads from CSV
TrafficGraph::TrafficGraph(const std::string& node_path, const std::string& road_path, const std::string& building_path) {
    load_from_csv(node_path, road_path, building_path);
}

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

// Load data and build the graph from the three CSV files
void TrafficGraph::load_from_csv(const std::string& node_path, const std::string& road_path, const std::string& building_path) {
    std::string line, word;

    // ==========================================
    // 1. Read nodes.csv (id, x, y)
    // ==========================================
    std::ifstream node_file(node_path);
    if (node_file.is_open()) {
        std::getline(node_file, line); // Skip the header line
        
        while (std::getline(node_file, line)) {
            if (line.empty()) continue;
            std::stringstream ss(line);
            std::string id, x_str, y_str;

            if (std::getline(ss, id, ',') && 
                std::getline(ss, x_str, ',') && 
                std::getline(ss, y_str, ',')) {
                
                try {
                    double x = std::stod(x_str);
                    double y = std::stod(y_str);
                    
                    Intersection* new_node = new Intersection(id, x, y);
                    intersections[id] = new_node;
                } catch (const std::exception& e) {
                    std::cerr << "Error parsing Intersection data: " << e.what() << " at line: " << line << std::endl;
                }
            }
        }
        node_file.close();
    } else {
        std::cerr << "Cannot open nodes.csv: " << node_path << std::endl;
    }

    // ==========================================
    // 2. Read roads.csv (id, source, target, lanes, speed_limit, is_oneway)
    // ==========================================
    std::ifstream road_file(road_path);
    if (road_file.is_open()) {
        std::getline(road_file, line); // Skip the header line
        
        while (std::getline(road_file, line)) {
            if (line.empty()) continue;
            std::stringstream ss(line);
            std::string edge_id, source_id, target_id, lanes_str, speed_str, is_oneway_str;

            if (std::getline(ss, edge_id, ',') &&
                std::getline(ss, source_id, ',') &&
                std::getline(ss, target_id, ',') &&
                std::getline(ss, lanes_str, ',') &&
                std::getline(ss, speed_str, ',') &&
                std::getline(ss, is_oneway_str, ',')) {
                
                try {
                    int lanes = std::stoi(lanes_str);
                    double speed_limit = std::stod(speed_str);
                    
                    // Parse the bool value ("1" = one-way, "0" = otherwise)
                    bool is_one_way = (is_oneway_str == "1");

                    // Only create the road if both source and target intersections exist
                    if (intersections.count(source_id) && intersections.count(target_id)) {
                        create_directional_road(edge_id, intersections[source_id], intersections[target_id],
                                                lanes, speed_limit, is_one_way);
                    } else {
                        std::cerr << "Warning: no Intersection found for edge " << edge_id << std::endl;
                    }
                } catch (const std::exception& e) {
                    std::cerr << "Error parsing Road data: " << e.what() << " at line: " << line << std::endl;
                }
            }
        }
        road_file.close();
    } else {
        std::cerr << "Cannot open roads.csv: " << road_path << std::endl;
    }

    // ==========================================
    // 3. Read buildings.csv (name, node_id, type)
    // ==========================================
    std::ifstream b_file(building_path);
    if (b_file.is_open()) {
        std::getline(b_file, line); // Skip the header line
        
        while (std::getline(b_file, line)) {
            if (line.empty()) continue;
            std::stringstream ss(line);
            std::string building_name, node_id, type;

            if (std::getline(ss, building_name, ',') &&
                std::getline(ss, node_id, ',') &&
                std::getline(ss, type, ',')) {
                
                if (intersections.count(node_id)) {
                    Building* new_building = new Building(building_name, node_id, type);
                    buildings[building_name] = new_building;
                } else {
                    std::cerr << "Warning: no linked Intersection found for Building " << building_name << std::endl;
                }
            }
        }
        b_file.close();
    } else {
        std::cerr << "Cannot open buildings.csv: " << building_path << std::endl;
    }
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
