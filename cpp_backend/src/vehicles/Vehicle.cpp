#include "vehicles/Vehicle.hpp"
#include "core/Road.hpp"
#include "core/Intersection.hpp"
#include "core/Graph.hpp"
#include "core/Building.hpp"
#include "core/TrafficLight.hpp"
#include "routing/AStar.hpp"
#include "routing/BFS.hpp"
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

// --- multi-lane and gridlock, used by the simulator-driven advance() -------
constexpr double kLaneWidth = 3.5;           // m between lane centres
constexpr double kLaneChangeCooldown = 5.0;  // s between two lane changes
constexpr double kLaneReturnInhibit = 12.0;  // s before flipping back is cheap
constexpr double kStuckSpeed = 0.3;          // m/s below which we count as stopped

// Is the slot `entry` metres into `road`, in `lane`, clear for a vehicle `len`
// metres long?
//
// The simulator picks every leader once per tick, before anything moves, so
// two vehicles arriving at a junction from different incoming roads both see
// an empty road ahead and both hand over onto it in the same tick. This check
// keeps the second one out: it runs during the move loop, so it sees whoever
// already claimed the slot.
bool entry_slot_free(const Road& road, int lane, double entry, double len) {
    for (const Vehicle* o : road.vehicles) {
        if (!o || o->lane != lane) continue;
        const double d = o->position_on_road - entry;
        // d >= 0: the other vehicle is in front, so its length is what matters.
        // d <  0: it is behind, so ours is.
        const double needed = (d >= 0.0 ? o->length_meters() : len) + kContactMargin;
        if (std::abs(d) < needed) return false;
    }
    return true;
}

// Build the concrete strategy behind a PathFinding pointer. The only place in
// the codebase that names Dijkstra, AStar or BFS directly.
std::unique_ptr<PathFinding> make_router(const std::string& name) {
    if (name == "astar") return std::make_unique<AStar>();
    if (name == "bfs")   return std::make_unique<BFS>();
    return std::make_unique<Dijkstra>();
}
}  // namespace

// Strategy pattern: the vehicle holds a PathFinding* and calls through the
// base class, so changing the algorithm swaps the object rather than picking a
// branch. Every routing call goes through here, so request_route() and
// recompute_route() can never disagree about which strategy is in use.
std::vector<Intersection*> Vehicle::run_router(const TrafficGraph& graph,
                                               const std::string& start_id,
                                               const std::string& target_id) const {
    return router_->find_path(graph, start_id, target_id);
}

void Vehicle::set_algorithm(const std::string& name) {
    if (name == algorithm_ && router_) return;  // already running this one
    algorithm_ = name;
    router_ = make_router(name);
}

Vehicle::Vehicle(std::string id, double max_speed, double max_accel, double max_decel)
    : id(id), current_road(nullptr), destination(nullptr), position_on_road(0.0),
      current_speed(0.0), max_speed(max_speed), accel(0.0), max_accel(max_accel),
      max_decel(max_decel), route_index(-1), is_reached_destination(false),
      router_(make_router(algorithm_)) {

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
    const double free_flow = std::min(max_speed, road.speed_limit);
    // Congestion slows everyone, but by how much depends on the vehicle: an
    // ambulance filters through, a bus does not. This is what gives
    // congestion_sensitivity() something to do.
    const double jam = road.get_congestion_level() * congestion_sensitivity();
    return free_flow * speed_factor() * idm_factor * std::max(0.25, 1.0 - 0.6 * jam);
}

void Vehicle::request_route(TrafficGraph* graph, Intersection* start, Intersection* target) {
    if (!graph || !start || !target) return;

    destination = target;
    is_reached_destination = false;

    std::vector<Intersection*> node_path = run_router(*graph, start->name, target->name);

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

    std::string start_node_id = current_road ? current_road->target->name : "";
    if (start_node_id.empty()) return;

    std::vector<Intersection*> node_path = run_router(*graph, start_node_id, target->name);
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

// ==========================================================================
// Hooks used by TrafficSimulator
// ==========================================================================

// ---- Observer -----------------------------------------------------------
bool Vehicle::remaining_route_contains(const std::string& road_id) const {
    for (std::size_t i = static_cast<std::size_t>(std::max(0, route_index));
         i < route.size(); ++i) {
        if (route[i] && route[i]->id == road_id) return true;
    }
    return false;
}

void Vehicle::onNotify(const TrafficNotification& notification) {
    // Only replan when the change can actually reach us. The typed payload is
    // what makes this possible: with a plain string every event would force
    // every vehicle in the city to recompute, even for a closure it is never
    // going to drive past.
    using Kind = TrafficNotification::Kind;
    switch (notification.kind) {
        case Kind::RoadClosed:
        case Kind::Accident:
        case Kind::Congestion:
            // Something got expensive on a road we still have to drive.
            if (notification.roadId.empty() ||
                remaining_route_contains(notification.roadId)) {
                needs_reroute_ = true;
            }
            return;

        case Kind::RoadReopened:
        case Kind::AccidentCleared:
        case Kind::CongestionCleared:
            // A shortcut may have come back; only worth a look if we are not
            // already driving through it.
            if (!notification.roadId.empty() &&
                !remaining_route_contains(notification.roadId)) {
                needs_reroute_ = true;
            }
            return;

        case Kind::TrafficLight:
        case Kind::TrafficLightRestored:
            // Signal timing does not change which roads exist, and the
            // car-following model already reacts to the light itself.
            return;
    }
}

bool Vehicle::consume_reroute_flag() {
    const bool flagged = needs_reroute_;
    needs_reroute_ = false;
    return flagged;
}

// ---- multi-lane ---------------------------------------------------------
bool Vehicle::lane_change_ready(double /*sim_time*/) const {
    // Mid-drift, or too soon after the last change: stay put.
    return current_road && lane_change_cooldown_ <= 0.0 &&
           std::abs(lateral_offset - target_lateral_offset) < 0.05;
}

void Vehicle::set_lane(int new_lane, double sim_time) {
    if (new_lane == lane) return;
    prev_lane_ = lane;
    prev_lane_blocked_until_ = sim_time + kLaneReturnInhibit;
    // Animate across: the drawn offset jumps to the old lane centre and then
    // eases back to zero, so the vehicle slides rather than teleports.
    lateral_offset += (lane - new_lane) * kLaneWidth;
    lane = new_lane;
    target_lateral_offset = 0.0;
    lane_change_cooldown_ = kLaneChangeCooldown;
}

// ---- gridlock relief ----------------------------------------------------
void Vehicle::rescue_to(std::size_t new_index) {
    if (new_index >= route.size()) return;
    Road* dest = route[new_index];
    if (!dest) return;

    int dest_lane = lane;
    const std::vector<int> ok = allowed_lanes(dest->lanes);
    if (std::find(ok.begin(), ok.end(), dest_lane) == ok.end()) {
        dest_lane = ok.empty() ? 0 : ok.front();
    }
    // Breaking the deadlock must not mean materialising inside someone else.
    // Leave the stuck counters alone so the next tick tries again.
    if (!entry_slot_free(*dest, dest_lane, 0.0, length_meters())) return;

    if (current_road) current_road->remove_vehicle(this);
    route_index = static_cast<int>(new_index);
    current_road = dest;
    position_on_road = 0.0;
    lane = dest_lane;
    current_speed = 0.0;
    accel = 0.0;
    stuck_time = 0.0;
    zone_wait_ = 0.0;
    current_road->add_vehicle(this);
}

// ---- simulator-driven movement ------------------------------------------
void Vehicle::advance(double dt, const Vehicle* lead, double gap, double sim_time) {
    if (is_reached_destination || !current_road) return;

    travel_time_ += dt;
    lane_change_cooldown_ = std::max(0.0, lane_change_cooldown_ - dt);

    // ---- 1. desired speed, capped by a red light ahead --------------------
    const double v0 = desired_speed_on(*current_road);
    double effective_gap = gap;
    const double dist_to_end = current_road->length - position_on_road;

    Intersection* next_node = current_road->target;
    const bool forced = is_forced(sim_time);
    if (!ignores_lights() && !forced && next_node && next_node->is_signalized &&
        next_node->light_system && dist_to_end < kLightLookahead) {
        const unsigned int light = next_node->light_system->get_state();
        if (light != 2) {  // red or amber: treat the stop line as a standing car
            effective_gap = std::min(effective_gap, std::max(0.1, dist_to_end));
            lead = nullptr;  // the "obstacle" is not moving
        }
    }

    // ---- 2. IDM acceleration ---------------------------------------------
    const double v_lead = lead ? lead->current_speed : 0.0;
    accel = idm_accel(current_speed, v0, v_lead, effective_gap);
    accel = std::clamp(accel, -max_decel, max_accel);

    current_speed = std::max(0.0, current_speed + accel * dt);

    // ---- 3. move, never through the car in front --------------------------
    double step = current_speed * dt;
    const double max_step = std::max(0.0, effective_gap - kContactMargin);
    if (step > max_step) {
        step = max_step;
        current_speed = dt > 1e-9 ? step / dt : 0.0;
    }
    position_on_road += step;
    last_gap = effective_gap;

    // ---- 4. stuck tracking ------------------------------------------------
    if (current_speed < kStuckSpeed && effective_gap < 1e8) {
        stuck_time += dt;
    } else if (current_speed > 1.0) {
        stuck_time = 0.0;
        zone_wait_ = 0.0;
    }

    // ---- 5. ease the lane-change drift ------------------------------------
    if (std::abs(lateral_offset - target_lateral_offset) > 1e-3) {
        const double d = target_lateral_offset - lateral_offset;
        const double m = kLateralSpeed * dt;
        lateral_offset += std::abs(d) <= m ? d : (d > 0 ? m : -m);
    }

    // ---- 6. hand over to the next road ------------------------------------
    if (position_on_road >= current_road->length) {
        const double overshoot = position_on_road - current_road->length;
        const int next_index = route_index + 1;

        if (next_index < static_cast<int>(route.size())) {
            Road* next = route[next_index];
            // Keep to a lane this vehicle is allowed to use on the new road.
            int next_lane = lane;
            const std::vector<int> ok = allowed_lanes(next->lanes);
            if (std::find(ok.begin(), ok.end(), next_lane) == ok.end()) {
                next_lane = ok.empty() ? 0 : ok.front();
            }

            // Someone may have taken the slot earlier in this same tick, after
            // the simulator handed us our leader. Hold at the stop line rather
            // than drive into the back of the queue.
            if (!entry_slot_free(*next, next_lane, overshoot, length_meters())) {
                position_on_road = current_road->length;
                current_speed = 0.0;
                accel = 0.0;
                return;
            }

            travel_history.push_back(current_road->id);
            current_road->remove_vehicle(this);
            route_index = next_index;
            current_road = next;
            position_on_road = overshoot;
            lane = next_lane;
            prev_lane_ = -1;
            current_road->add_vehicle(this);
        } else {
            travel_history.push_back(current_road->id);
            current_road->remove_vehicle(this);
            route_index = next_index;
            is_reached_destination = true;
            current_road = nullptr;
            position_on_road = 0.0;
            current_speed = 0.0;
            accel = 0.0;
        }
    }
}
