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

// Congestion drags the free-flow speed down to 15% of the limit at worst, but
// never to a standstill: a road reporting 0 m/s would make travel_time_seconds()
// infinite and IDM's target speed zero, so traffic could never crawl out of the
// jam it is in. `sensitivity` is how much this vehicle is affected -- 1.0 for
// ordinary traffic, lower for an ambulance filtering through.
double Road::effective_speed_ms(double sensitivity) const {
    const double slowdown =
        1.0 - 0.85 * congestion_level * std::clamp(sensitivity, 0.0, 1.0);
    return std::max(1.0, speed_limit_ms() * slowdown);  // never below 1 m/s
}

double Road::travel_time_seconds(double sensitivity) const {
    return length / effective_speed_ms(sensitivity);   // speed is never 0
}

double Road::stop_line_position() const {
    if (signal_group < 0) return length;   // no signal: nothing to stop for
    // Never let the line fall off the start of a short block, or a vehicle
    // would be past it the moment it enters and would never stop.
    return std::max(0.0, length - signal_stop_from_end);
}

double Road::cost(CostMode mode) const {
    if (is_closed) return std::numeric_limits<double>::infinity();
    return mode == CostMode::Distance ? length : travel_time_seconds();
}

void Road::update_travel_cost() {
    // If road is closed, set travel cost to infinity which prevent pathfinding algorithms from routing onto this road
    if (is_closed) {
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
