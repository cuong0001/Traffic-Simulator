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
#include <cctype>
#include <cstdlib>
#include <iostream>
#include <cmath>

namespace {
// Intelligent Driver Model constants (Treiber et al.).
constexpr double kIdmT = 0.25;             // desired safe time headway (s) for tight urban platoons
constexpr double kIdmS0 = 1.0;              // minimum standstill gap (m)
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
    : id(id), current_road(nullptr), position_on_road(0.0), current_speed(0.0),
      destination_(nullptr), max_speed_(max_speed), accel_(0.0), max_accel_(max_accel),
      max_decel_(max_decel), route_index_(-1), is_reached_destination_(false),
      router_(make_router(algorithm_)) {

    // Derive the speed personality from a hash of the id, so identical cars
    // do not drive identically and a run stays reproducible.
    unsigned h = 0;
    for (char c : id) h = h * 31u + static_cast<unsigned>(c);
    idm_factor = 0.8 + 0.2 * ((h % 100u) / 99.0);

    // Vehicle ids are always "<type><counter>" (TrafficSimulator::spawn_locked),
    // so the counter is the maximal trailing digit run -- this reconstructs
    // correctly regardless of which vehicle type prefixes it, unlike a fixed
    // 'v' prefix check (which never matched: real prefixes are car/bus/
    // emergency/motorbike). The frontend rebuilds the display id as
    // TYPE_NAMES[type] + num_id from the binary snapshot, so this has to stay
    // reversible; the hash fallback below only fires for a hand-built id with
    // no trailing digits at all, which spawn_locked never produces.
    std::size_t digits_start = id.size();
    while (digits_start > 0 && std::isdigit(static_cast<unsigned char>(id[digits_start - 1]))) {
        --digits_start;
    }
    num_id = digits_start < id.size()
                 ? static_cast<uint32_t>(std::strtoul(id.c_str() + digits_start, nullptr, 10))
                 : static_cast<uint32_t>(h);
}

// Intelligent Driver Model acceleration.
double Vehicle::idm_accel(double v, double v0, double v_lead, double gap) {
    v0 = std::max(0.5, v0);
    gap = std::max(0.1, gap);
    const double delta_v = v - v_lead;
    const double sqrt_ab = 2.0 * std::sqrt(kIdmA * kIdmB);
    const double s_star = kIdmS0 + std::max(0.0, v * kIdmT + (v * delta_v) / sqrt_ab);
    double a = kIdmA * (1.0 - std::pow(v / v0, 4) - std::pow(s_star / gap, 2));

    // Crisp launch: when accelerating from low speed into a clear gap,
    // apply full acceleration instead of lingering.
    if (v < 3.0 && gap > 2.0 * s_star && v_lead >= v) {
        a = std::max(a, kIdmA * 0.95);
    }
    return a;
}

double Vehicle::desired_speed_on(const Road& road) const {
    const double free_flow =
        std::min(max_speed_, road.effective_speed_ms(congestion_sensitivity()));
    return free_flow * speed_factor() * idm_factor;
}

void Vehicle::request_route(TrafficGraph* graph, Intersection* start, Intersection* target) {
    if (!graph || !start || !target) return;

    destination_ = target;
    is_reached_destination_ = false;

    std::vector<Intersection*> node_path = run_router(*graph, start->name, target->name);

    route_.clear();

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
            route_.push_back(found_road);
        }
    }

    if (!route_.empty()) {
        route_index_ = 0;
        current_road = route_[route_index_];
        position_on_road = 0.0;
        current_road->add_vehicle(this);
    } else {
        route_index_ = -1;
        current_road = nullptr;
    }
}



void Vehicle::recompute_route(TrafficGraph* graph, Intersection* target) {
    if (!graph || !target) return;

    destination_ = target;

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
        route_.clear();
        route_.push_back(saved_current);
        route_.insert(route_.end(), new_route.begin(), new_route.end());
        route_index_ = 0;
        is_reached_destination_ = false;
        route_dirty_ = false;
        recompute_cooldown_ = 5.0;  // Minimum 5s cooldown between reroutes
        ++recalculations_;
    }
}

// Getters
std::string Vehicle::get_id() const { return id; }
bool Vehicle::has_reached_destination() const { return is_reached_destination_; }
Road* Vehicle::get_current_road() const { return current_road; }
Intersection* Vehicle::get_destination() const { return destination_; }
double Vehicle::get_position_on_road() const { return position_on_road; }
const std::vector<std::string>& Vehicle::get_travel_history() const { return travel_history_; }
const std::vector<Road*>& Vehicle::get_route() const { return route_; }
int Vehicle::get_route_index() const { return route_index_; }
double Vehicle::get_current_speed() const { return current_speed; }
double Vehicle::get_max_speed() const { return max_speed_; }
double Vehicle::get_acceleration() const { return accel_; }
double Vehicle::get_max_acceleration() const { return max_accel_; }
double Vehicle::get_max_deceleration() const { return max_decel_; }
std::size_t Vehicle::last_nodes_explored() const {
    return router_ ? router_->nodes_explored() : 0;
}

// ==========================================================================
// Hooks used by TrafficSimulator
// ==========================================================================

// ---- Observer -----------------------------------------------------------
bool Vehicle::remaining_route_contains(const std::string& road_id) const {
    for (std::size_t i = static_cast<std::size_t>(std::max(0, route_index_));
         i < route_.size(); ++i) {
        if (route_[i] && route_[i]->id == road_id) return true;
    }
    return false;
}

void Vehicle::onNotify(const TrafficNotification& notification) {
    using Kind = TrafficNotification::Kind;
    switch (notification.kind) {
        case Kind::RoadClosed:
        case Kind::Accident:
        case Kind::Congestion:
            if (notification.roadId.empty() ||
                remaining_route_contains(notification.roadId)) {
                needs_reroute_ = true;
            }
            return;

        case Kind::RoadReopened:
        case Kind::AccidentCleared:
        case Kind::CongestionCleared:
            if (!notification.roadId.empty() &&
                !remaining_route_contains(notification.roadId)) {
                needs_reroute_ = true;
                const std::size_t h = std::hash<std::string>{}(id);
                const double stagger = 1.0 + static_cast<double>(h % 100) / 10.0;
                recompute_cooldown_ = std::max(recompute_cooldown_, stagger);
            }
            return;

        case Kind::TrafficLight:
        case Kind::TrafficLightRestored:
            return;
    }
}

bool Vehicle::consume_reroute_flag() {
    const bool flagged = needs_reroute_;
    if (flagged) needs_reroute_ = false;
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

void Vehicle::transfer_to_road(Road* new_road, double progress_meters) {
    if (!new_road) return;

    // The caller has already detached us from the old road -- it no longer
    // exists, so there is nothing to remove ourselves from. Doing it here
    // anyway would dereference a deleted Road.
    current_road = new_road;
    position_on_road = std::clamp(progress_meters, 0.0, new_road->length);

    // Keep the motion state. The whole point of transferring rather than
    // respawning is that the driver does not notice the edit: same speed, same
    // lane if that lane still exists, same sideways drift.
    const std::vector<int> ok = allowed_lanes(new_road->get_lanes());
    if (std::find(ok.begin(), ok.end(), lane) == ok.end()) {
        lane = ok.empty() ? 0 : ok.front();
        target_lateral_offset = 0.0;
    }

    // The old route ran along a road that is gone. Keep only where we are and
    // ask for a replan; the simulator picks it up on the next tick.
    route_.clear();
    route_.push_back(new_road);
    route_index_ = 0;
    needs_reroute_ = true;

    new_road->add_vehicle(this);
}

// ---- gridlock relief ----------------------------------------------------
bool Vehicle::rescue_to(std::size_t new_index) {
    if (new_index >= route_.size()) return false;
    Road* dest = route_[new_index];
    if (!dest) return false;

    int dest_lane = lane;
    const std::vector<int> ok = allowed_lanes(dest->get_lanes());
    if (std::find(ok.begin(), ok.end(), dest_lane) == ok.end()) {
        dest_lane = ok.empty() ? 0 : ok.front();
    }
    // Breaking the deadlock must not mean materialising inside someone else.
    // Leave the stuck counters alone so the next tick tries again.
    if (!entry_slot_free(*dest, dest_lane, 0.0, length_meters())) return false;

    if (current_road) current_road->remove_vehicle(this);
    route_index_ = static_cast<int>(new_index);
    current_road = dest;
    position_on_road = 0.0;
    lane = dest_lane;
    current_speed = 0.0;
    accel_ = 0.0;
    stuck_time = 0.0;
    zone_wait_ = 0.0;
    current_road->add_vehicle(this);
    return true;
}

// ---- simulator-driven movement ------------------------------------------
void Vehicle::advance(double dt, const Vehicle* lead, double gap, double sim_time) {
    if (is_reached_destination_ || !current_road) return;

    travel_time_ += dt;
    lane_change_cooldown_ = std::max(0.0, lane_change_cooldown_ - dt);
    recompute_cooldown_ = std::max(0.0, recompute_cooldown_ - dt);

    // ---- 1. desired speed, capped by a red light ahead --------------------
    const double v0 = desired_speed_on(*current_road);
    double effective_gap = gap;

    Intersection* next_node = current_road->target;
    const bool forced = is_forced(sim_time);
    if (!ignores_lights() && !forced && next_node && next_node->is_signalized() &&
        current_road->signal_group >= 0) {
        // The stop line sits a few metres BEFORE the junction, so a vehicle
        // held on red waits at the line instead of standing in the box. The
        // colour is the one shown to THIS approach's phase group: the crossing
        // street is showing the opposite colour at the same moment.
        const double dist_to_stop = current_road->stop_line_position() - position_on_road;
        if (dist_to_stop < kLightLookahead) {
            const unsigned int light =
                next_node->signal_state_for(current_road->signal_group, sim_time);
            bool must_stop = (light == TrafficLight::Red);

            // Amber is a decision, not a colour. Treating it as red is what
            // puts a queue at every junction: a vehicle already too close to
            // brake comfortably should carry on through, as a driver would.
            // kAmberComfortDecel is the threshold react_to_congestion() has
            // always used for exactly this call.
            if (light == TrafficLight::Amber) {
                const double d = std::max(0.5, dist_to_stop);
                must_stop =
                    (current_speed * current_speed) / (2.0 * d) <= kAmberComfortDecel;
            }

            if (must_stop) {
                const double stop_gap = dist_to_stop - length_meters() * 0.5;
                if (stop_gap > -1.0) {  // not already past the line
                    effective_gap = std::min(effective_gap, std::max(0.1, stop_gap));
                    lead = nullptr;  // the "obstacle" is not moving
                }
            }
        }
    }

    // Record WHY we are about to move or not, before the physics runs. A red
    // light is checked first because it outranks everything else: a vehicle
    // stopped at a light is not blocked, it is obeying.
    if (is_reached_destination_) {
        state_ = State::Arrived;
    } else if (route_.empty()) {
        state_ = State::Planning;
    } else if (effective_gap < gap) {
        // effective_gap was shortened above, which only happens for a signal.
        state_ = State::WaitingAtLight;
    } else if (current_speed < 0.5 && effective_gap < 2.0 * length_meters()) {
        state_ = State::Blocked;
    } else {
        state_ = State::Moving;
    }

    // ---- 2. IDM acceleration ---------------------------------------------
    const double v_lead = lead ? lead->current_speed : 0.0;
    accel_ = idm_accel(current_speed, v0, v_lead, effective_gap);
    accel_ = std::clamp(accel_, -max_decel_, max_accel_);

    current_speed = std::max(0.0, current_speed + accel_ * dt);

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
        const int next_index = route_index_ + 1;

        if (next_index < static_cast<int>(route_.size())) {
            Road* next = route_[next_index];
            // Keep to a lane this vehicle is allowed to use on the new road.
            int next_lane = lane;
            const std::vector<int> ok = allowed_lanes(next->get_lanes());
            if (std::find(ok.begin(), ok.end(), next_lane) == ok.end()) {
                next_lane = ok.empty() ? 0 : ok.back();
            }

            // Someone may have taken the slot earlier in this same tick, after
            // the simulator handed us our leader. Hold at the stop line rather
            // than drive into the back of the queue.
            if (!entry_slot_free(*next, next_lane, overshoot, length_meters())) {
                position_on_road = current_road->length;
                current_speed = 0.0;
                accel_ = 0.0;
                return;
            }

            travel_history_.push_back(current_road->id);
            current_road->remove_vehicle(this);
            route_index_ = next_index;
            current_road = next;
            position_on_road = overshoot;
            lane = next_lane;
            prev_lane_ = -1;
            current_road->add_vehicle(this);
        } else {
            travel_history_.push_back(current_road->id);
            current_road->remove_vehicle(this);
            route_index_ = next_index;
            is_reached_destination_ = true;
            state_ = State::Arrived;
            current_road = nullptr;
            position_on_road = 0.0;
            current_speed = 0.0;
            accel_ = 0.0;
        }
    }
}
