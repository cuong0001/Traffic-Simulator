#pragma once
#include <string>
#include <utility>
#include <vector>

class Building {
public:
    // Building with a footprint loaded from the map file
    Building(std::string name, std::string type, int levels,
             std::vector<std::pair<double, double>> footprint);

    const std::string& get_name() const { return name_; }
    const std::string& get_type() const { return type_; }        // "commercial", "hospital", "residential", ...
    int get_levels() const { return levels_; }                   // Number of floors (used when drawing)

    // Building outline (polygon) in world coordinates.
    // Empty when the building is only anchored to a node.
    const std::vector<std::pair<double, double>>& get_footprint() const { return footprint_; }

    // Intersection this building is anchored to ("" if display-only). Not a
    // constructor parameter: MapLoader sets it once, right after
    // construction, only when the map file specifies one -- a setter (rather
    // than a public field) still means nothing else in the codebase can
    // reassign a building's anchor later.
    const std::string& get_node_id() const { return node_id_; }
    void set_node_id(std::string node_id) { node_id_ = std::move(node_id); }

    // Centroid of the footprint; returns (0, 0) when there is no footprint.
    std::pair<double, double> centroid() const;

private:
    std::string name_;
    std::string node_id_;
    std::string type_;
    int levels_;
    std::vector<std::pair<double, double>> footprint_;
};
