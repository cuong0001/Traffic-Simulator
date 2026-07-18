#include "../../include/Vehicle.h"
#include "../../include/Road.h"
#include "../../include/Intersection.h"
#include "../../include/Graph.h"
#include "../../include/Building.h"
#include "../../include/Dijkstra.h"
#include <algorithm>
#include <iostream>

Vehicle::Vehicle(std::string id, double max_speed, double max_accel, double max_decel)
    : id(id), current_road(nullptr), destination(nullptr), position_on_road(0.0),
      current_speed(0.0), max_speed(max_speed), accel(0.0), max_accel(max_accel),
      max_decel(max_decel), route_index(-1), is_reached_destination(false) {}

void Vehicle::request_route(TrafficGraph* graph, Intersection* start, Intersection* target) {
    if (!graph || !start || !target) return;

    destination = target;
    is_reached_destination = false;

    Dijkstra router;
    std::vector<Intersection*> node_path = router.find_path(*graph, start->name, target->name);

    route.clear();

    // List roads based on the node path
    for (size_t i = 0; i < node_path.size() - 1; ++i) {
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

    // Initialize the vehicle's position on the first road of the route
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

    // React to congestion and adjust acceleration accordingly
    react_to_congestion();

    // Update the vehicle's speed based on acceleration and time step
    current_speed += accel * dt;
    double allowed_max_speed = std::min(max_speed, current_road->speed_limit);
    if (current_speed > allowed_max_speed) current_speed = allowed_max_speed;
    if (current_speed < 0.0) current_speed = 0.0;

    // Update the vehicle's position on the current road
    position_on_road += current_speed * dt;

    // Change to the next road if the vehicle has reached the end of the current road
    if (position_on_road >= current_road->length) {
        double overshoot = position_on_road - current_road->length; // Keep the overshoot distance for the next road
        
        travel_history.push_back(current_road->id); // Add the current road to the travel history
        current_road->remove_vehicle(this); // Remove the vehicle from the current road's vehicle list because it is moving to the next road

        route_index++; // Move to the next road in the route

        if (route_index < static_cast<int>(route.size())) {
            current_road = route[route_index];
            position_on_road = overshoot;   // Put the overshoot distance on the next road
            current_road->add_vehicle(this);
        } else {
            // Reached the destination
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
    double safe_distance = 15.0; // Safe distance to maintain from the lead vehicle
    double stopping_distance = 25.0; // Distance to start slowing down for traffic lights or obstacles

    //  Check for lead vehicle or obstacles on the current road
    std::pair<Vehicle*, double> lead = current_road->get_lead_vehicle(this);
    
    if (lead.first != nullptr && lead.second < safe_distance) {
        accel = -max_decel;
        is_congested = true;
        return is_congested;
    }

    // Check for traffic light at the end of the current road
    Intersection* next_node = current_road->target;
    if (next_node && next_node->light_system) {
        double distance_to_intersection = current_road->length - position_on_road;

        unsigned int light_state = next_node->light_system->get_state();
        unsigned int remaining_time = next_node->light_system->get_remaining_time();

        if (distance_to_intersection < stopping_distance && distance_to_intersection > 0 && light_state != 2) {
            //  If the light is red or yellow and the vehicle is within stopping distance, start decelerating
            accel = -max_decel * (1.0 - (distance_to_intersection / stopping_distance));
            if (accel > 0) accel = -max_decel; // Ensure deceleration is negative
            is_congested = true;
            return is_congested;
        }
    }

    // Check if the vehicle is exceeding the speed limit of the current road
    double allowed_max_speed = std::min(max_speed, current_road->speed_limit);
    if (current_speed < allowed_max_speed) {
        accel = max_accel;
    } else {
        accel = 0.0;
    }

    // Check for congestion on the current road
    if (current_road->get_congestion_level() > 0.7) {
        is_congested = true;
    }

    return is_congested;
}

void Vehicle::recompute_route(TrafficGraph* graph, Intersection* target) {
    if (!graph || !target) return;

    destination = target;
    Dijkstra router;
    
    // If the vehicle is currently on a road, use the target node of that road as the starting point for the new route. 
    // Otherwise, use the vehicle's current position (if available) or return early if no valid starting point exists.
    std::string start_node_id = current_road ? current_road->target->name : "";
    if (start_node_id.empty()) return;

    std::vector<Intersection*> node_path = router.find_path(*graph, start_node_id, target->name);
    std::vector<Road*> new_route;

    // List roads based on the node path
    for (size_t i = 0; i < node_path.size() - 1; ++i) {
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
        // If the new route is valid, update the vehicle's route while keeping the current road as the first road in the new route.
        Road* saved_current = current_road;
        route.clear();
        route.push_back(saved_current); // Keep the current road as the first road in the new route
        route.insert(route.end(), new_route.begin(), new_route.end()); // Insert the new optimal route after the current road
        route_index = 0; 
        is_reached_destination = false;
    }
}

//Getter
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