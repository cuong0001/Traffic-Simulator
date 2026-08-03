#include "core/Building.hpp"
#include <utility>

Building::Building(std::string name, std::string type, int levels,
                   std::vector<std::pair<double, double>> footprint)
    : name_(std::move(name)), type_(std::move(type)), levels_(levels),
      footprint_(std::move(footprint)) {}

std::pair<double, double> Building::centroid() const {
    if (footprint_.empty()) return {0.0, 0.0};

    double sum_x = 0.0, sum_y = 0.0;
    for (const auto& point : footprint_) {
        sum_x += point.first;
        sum_y += point.second;
    }
    const double n = static_cast<double>(footprint_.size());
    return {sum_x / n, sum_y / n};
}
