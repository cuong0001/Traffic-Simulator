#pragma once
#include <string>
#include <utility>
#include <vector>

class Building {
public:
    std::string name;
    std::string node_id;  // Intersection the building is anchored to ("" if display-only)
    std::string type;     // "commercial", "hospital", "residential", ...
    int levels;           // Number of floors (used when drawing)

    // Building outline (polygon) in world coordinates.
    // Empty when the building is only anchored to a node.
    std::vector<std::pair<double, double>> footprint;

    // Building anchored to a node (old CSV format)
    Building(std::string name, std::string node_id, std::string type);

    // Building with a footprint loaded from the map file
    Building(std::string name, std::string type, int levels,
             std::vector<std::pair<double, double>> footprint);

    // Centroid of the footprint; returns (0, 0) when there is no footprint.
    std::pair<double, double> centroid() const;
};
