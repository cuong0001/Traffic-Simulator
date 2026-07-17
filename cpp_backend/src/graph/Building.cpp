#include "../../include/Building.h"
#include <utility>

Building::Building(std::string name, std::string node_id, std::string type)
    : name(std::move(name)), node_id(std::move(node_id)), type(std::move(type)),
      levels(1) {}

Building::Building(std::string name, std::string type, int levels,
                   std::vector<std::pair<double, double>> footprint)
    : name(std::move(name)), type(std::move(type)), levels(levels),
      footprint(std::move(footprint)) {}

std::pair<double, double> Building::centroid() const {
    if (footprint.empty()) return {0.0, 0.0};

    double sum_x = 0.0, sum_y = 0.0;
    for (const auto& point : footprint) {
        sum_x += point.first;
        sum_y += point.second;
    }
    const double n = static_cast<double>(footprint.size());
    return {sum_x / n, sum_y / n};
}
