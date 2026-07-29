#include "vehicles/Vehicle.hpp"
#include "core/Road.hpp"
#include "core/Intersection.hpp"
#include "core/Graph.hpp"
#include "core/Building.hpp"
#include "routing/Dijkstra.hpp"
#include <algorithm>
#include <iostream>
#include <cmath>

namespace {
// Intelligent Driver Model constants (Treiber et al.).
constexpr double kIdmT = 1.5;              // desired safe time headway (s)
constexpr double kIdmS0 = 2.0;             // minimum standstill gap (m)
constexpr double kIdmA = 2.5;              // comfortable acceleration (m/s^2)
constexpr double kIdmB = 3.0;              // comfortable deceleration (m/s^2)
constexpr double kContactMargin = 0.25;    // m kept between bumpers
constexpr double kLightLookahead = 80.0;   // m before a stop line we start looking
constexpr double kAmberComfortDecel = 3.0; // amber dilemma-zone threshold (m/s^2)
constexpr double kLateralSpeed = 2.0;      // m/s of sideways drift on a lane change
}  // namespace

Vehicle::Vehicle(std::string id, double max_speed, double max_accel, double max_decel)
    : id(id), current_road(nullptr), destination(nullptr), position_on_road(0.0),
      current_speed(0.0), max_speed(max_speed), accel(0.0), max_accel(max_accel),
      max_decel(max_decel), route_index(-1), is_reached_destination(false) {
    
    // Derive the speed personality from a hash of the id, so identical cars
    // do not drive identically and a run stays reproducible.
    unsigned h = 0;
    for (char c : id) h = h * 31u + static_cast<unsigned>(c);
    idm_factor = 0.8 + 0.2 * ((h % 100u) / 99.0);
}

// Intelligent Driver Model acceleration.
//   v     current speed (m/s)      v0   desired free-flow speed (m/s)
//   v_lead speed of the car ahead  gap  bumper-to-bumper distance (m)
double Vehicle::idm_accel(double v, double v0, double v_lead, double gap) {
    v0 = std::max(0.5, v0);
    gap = std::max(0.1, gap);
    const double delta_v = v - v_lead;
    const double sqrt_ab = 2.0 * std::sqrt(kIdmA * kIdmB);
    const double s_star = kIdmS0 + std::max(0.0, v * kIdmT + (v * delta_v) / sqrt_ab);
    return kIdmA * (1.0 - std::pow(v / v0, 4) - std::pow(s_star / gap, 2));
}

double Vehicle::desired_speed_on(const Road& road) const {
    double road_limit = road.speed_limit;
    return std::min(max_speed, road_limit) * speed_factor() * idm_factor;
}

void Vehicle::request_route(TrafficGraph* graph, Intersection* start, Intersection* target) {
    if (!graph || !start || !target) return;

    destination = target;
    is_reached_destination = false;

    Dijkstra router;
    std::vector<Intersection*> node_path = router.find_path(*graph, start->name, target->name);

    route.clear();

    for (size_t i = 0; i + 1 < node_path.size(); ++i) {
        Intersection* u = node_path[i];
        Intersection* v = node_path[i + 1];
        Road* found_road = nullptr;

        for (Road* road : u->outgoing_edges) {
            if (road->target == v) {
                found_road = road;
                break;
            }
        }
        if (found_road) {
            route.push_back(found_road);
        }
    }

    if (!route.empty()) {
        route_index = 0;
        current_road = route[route_index];
        position_on_road = 0.0;
        current_road->add_vehicle(this);
    } else {
        route_index = -1;
        current_road = nullptr;
    }
}

void Vehicle::step_by_step(double dt) {
    if (is_reached_destination || !current_road) return;

    // React to congestion & IDM car-following calculation
    react_to_congestion();

    // Apply the IDM acceleration.
    current_speed = std::max(0.0, current_speed + accel * dt);
    double step = current_speed * dt;

    // Never move through the vehicle in front. IDM is a comfort model: it does
    // not guarantee this when the leader brakes hard or dt is large.
    const double max_step = std::max(0.0, last_gap - kContactMargin);
    if (step > max_step) {
        step = max_step;
        current_speed = dt > 1e-9 ? step / dt : 0.0;
    }
    position_on_road += step;

    // Ease the sideways drift of a lane change.
    if (current_speed > 0.5) {
        double delta = target_lateral_offset - lateral_offset;
        double max_step = kLateralSpeed * dt;
        if (std::abs(delta) <= max_step) lateral_offset = target_lateral_offset;
        else lateral_offset += (delta > 0 ? max_step : -max_step);
    }

    // Stuck tracking: how long have we been standing still?
    if (current_speed < 0.3) stuck_time += dt;
    else if (current_speed > 1.0) stuck_time = 0.0;

    // Hand over to the next road once this one runs out.
    if (position_on_road >= current_road->length) {
        double overshoot = position_on_road - current_road->length;
        
        travel_history.push_back(current_road->id);
        current_road->remove_vehicle(this);

        route_index++;

        if (route_index < static_cast<int>(route.size())) {
            current_road = route[route_index];
            position_on_road = std::min(overshoot, current_road->length);
            current_road->add_vehicle(this);
        } else {
            // Arrived at the destination.
            is_reached_destination = true;
            current_road = nullptr;
            position_on_road = 0.0;
            current_speed = 0.0;
            accel = 0.0;
        }
    }
}

bool Vehicle::react_to_congestion() {
    if (!current_road) return false;

    bool is_congested = false;
    double v0 = desired_speed_on(*current_road);

    // 1. The vehicle ahead and the distance to it.
    // TODO: this is lane-blind; the lane-aware search lives in TrafficSimulator.
    std::pair<Vehicle*, double> lead = current_road->get_lead_vehicle(this);
    // get_lead_vehicle() measures position to position; IDM needs bumper to bumper.
    double gap = lead.first ? lead.second - lead.first->length_meters() : 1e9;
    double v_lead = lead.first ? lead.first->get_current_speed() : 0.0;

    bool red_ahead = false;

    // 2. Traffic light ahead, including the amber dilemma zone.
    if (!ignores_lights()) {
        Intersection* next_node = current_road->target;
        if (next_node && next_node->light_system) {
            double dist_to_stop = current_road->length - position_on_road;
            if (dist_to_stop < kLightLookahead) {
                unsigned int state = next_node->light_system->get_state(); // 0: Red, 1: Yellow, 2: Green
                bool must_stop = (state == 0);

                // Amber: stop only if we can still brake comfortably.
                if (state == 1) {
                    double d = std::max(0.5, dist_to_stop);
                    must_stop = (current_speed * current_speed) / (2.0 * d) <= kAmberComfortDecel;
                }

                if (must_stop) {
                    double stop_gap = dist_to_stop - length_meters() * 0.5;
                    if (stop_gap > -1.0) {  // not past the stop line yet
                        red_ahead = true;
                        if (stop_gap < gap) {
                            gap = std::max(0.05, stop_gap);
                            v_lead = 0.0;
                        }
                    }
                }
            }
        }
    }

    // 3. IDM acceleration, clamped to what this vehicle can actually do.
    accel = idm_accel(current_speed, v0, v_lead, gap);
    accel = std::clamp(accel, -max_decel, max_accel);

    last_gap = gap;   // step_by_step() uses this so we never move through the leader

    if (gap < 15.0 || red_ahead) {
        is_congested = true;
    }

    return is_congested;
}

void Vehicle::recompute_route(TrafficGraph* graph, Intersection* target) {
    if (!graph || !target) return;

    destination = target;
    Dijkstra router;
    
    std::string start_node_id = current_road ? current_road->target->name : "";
    if (start_node_id.empty()) return;

    std::vector<Intersection*> node_path = router.find_path(*graph, start_node_id, target->name);
    std::vector<Road*> new_route;

    for (size_t i = 0; i + 1 < node_path.size(); ++i) {
        Intersection* u = node_path[i];
        Intersection* v = node_path[i + 1];
        Road* found_road = nullptr;
        for (Road* road : u->outgoing_edges) {
            if (road->target == v) {
                found_road = road;
                break;
            }
        }
        if (found_road) new_route.push_back(found_road);
    }

    if (!new_route.empty()) {
        Road* saved_current = current_road;
        route.clear();
        route.push_back(saved_current);
        route.insert(route.end(), new_route.begin(), new_route.end());
        route_index = 0; 
        is_reached_destination = false;
    }
}

// Getters
std::string Vehicle::get_id() const { return id; }
bool Vehicle::has_reached_destination() const { return is_reached_destination; }
Road* Vehicle::get_current_road() const { return current_road; }
double Vehicle::get_position_on_road() const { return position_on_road; }
const std::vector<std::string>& Vehicle::get_travel_history() const { return travel_history; }
const std::vector<Road*>& Vehicle::get_route() const { return route; }
int Vehicle::get_route_index() const { return route_index; }
double Vehicle::get_current_speed() const { return current_speed; }
double Vehicle::get_max_speed() const { return max_speed; }
double Vehicle::get_acceleration() const { return accel; }
double Vehicle::get_max_acceleration() const { return max_accel; }
double Vehicle::get_max_deceleration() const { return max_decel; }
