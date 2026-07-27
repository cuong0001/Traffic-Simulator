#include "core/Road.hpp"
#include "core/Intersection.hpp"
#include "vehicles/Vehicle.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

Road::Road(std::string id, Intersection* source, Intersection* target, int lanes, double speed_limit,
           int capacity, bool is_one_way, double length, Geometry* geometry)
    : id(std::move(id)), source(source), target(target),
      lanes(lanes), speed_limit(speed_limit),
      capacity(capacity), is_one_way(is_one_way),
      is_internal(false), congestion_level(0.0), geometry(geometry) {

    // Length comes from the map file when given; otherwise measure it from the
    // endpoints.
    if (length > 0.0) {
        this->length = length;
    } else {
        double dx = target->x - source->x;
        double dy = target->y - source->y;
        this->length = std::sqrt(dx * dx + dy * dy);
    }

    // base_weight = cost of traversing the whole road under ideal conditions
    base_weight = this->length;
    travel_cost = base_weight;

    // Add this edge to both intersections' adjacency lists for graph traversal (pathfinding)
    source->outgoing_edges.push_back(this);
    target->incoming_edges.push_back(this);
}

void Road::add_vehicle(Vehicle* vehicle) {
    vehicles.push_back(vehicle);
}

void Road::remove_vehicle(Vehicle* vehicle) {
    vehicles.erase(std::remove(vehicles.begin(), vehicles.end(), vehicle), vehicles.end());
}

double Road::get_congestion_level() const {
    return congestion_level;
}

double Road::get_weight() const {
    return travel_cost;
}

// Update travel cost based on the current congestion level
void Road::update() {
    update_travel_cost();
}

// Find the nearest vehicle ahead of current_vehicle on this road (the one with
// the smallest position_on_road among vehicles ahead of current_vehicle).
// Returns (lead vehicle, distance to it).
// If the road ahead is clear: returns (nullptr, +infinity).
// If current_vehicle is nullptr (a vehicle about to enter this road): measured from the road start.
std::pair<Vehicle*, double> Road::get_lead_vehicle(Vehicle* current_vehicle) {
    double ref_position = current_vehicle ? current_vehicle->position_on_road : 0.0;

    Vehicle* lead = nullptr;
    double min_gap = std::numeric_limits<double>::infinity();

    for (Vehicle* other : vehicles) {
        if (!other || other == current_vehicle) continue;

        double gap = other->position_on_road - ref_position;
        if (gap > 0.0 && gap < min_gap) {
            min_gap = gap;
            lead = other;
        }
    }

    return {lead, min_gap};
}

void Road::update_travel_cost() {
    // If road is closed, set travel cost to infinity which prevent pathfinding algorithms from routing onto this road
    if (is_closed) {
        static constexpr double kClosedRoadCost = 1e8;
        travel_cost = kClosedRoadCost;
        return;
    }

    // Calculating the composite traffic congestion level: Congestion due to current traffic volume + Congestion due to events.
    // capacity_scale < 1.0 (accident) increase 
    const double actual_capacity = std::max(1.0, lanes * capacity * capacity_scale);
    double base_congestion = vehicles.size() / actual_capacity;
    
    congestion_level = std::min(1.0, base_congestion + event_congestion);

    // Recomputed travel cost.
    travel_cost = base_weight * (1.0 + congestion_level);
}
