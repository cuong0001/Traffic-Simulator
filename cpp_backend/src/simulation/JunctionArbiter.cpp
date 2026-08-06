#include "simulation/JunctionArbiter.hpp"

#include <algorithm>
#include <cmath>

#include "core/Graph.hpp"
#include "core/Intersection.hpp"
#include "core/Road.hpp"
#include "simulation/TrafficController.hpp"
#include "vehicles/Vehicle.hpp"

namespace {
    // ---- MOBIL (lane-change decision model) ---------------------------------
    constexpr double kMobilSafeDecel = -2.0;       // new follower may brake this hard
    constexpr double kMobilThreshold = 0.35;       // min gain to bother (m/s^2)
    constexpr double kMobilReturnThreshold = 1.0;  // going back needs a bigger win
    constexpr double kMobilObstruction = 0.4;      // only change if actually held up
    // Clear bumper-to-bumper room required in the target lane, ahead AND
    // behind, before a lane change is even considered -- checked separately
    // from the gain math below, which is purely relative ("is this lane better
    // than mine") and so can accept an objectively occupied lane when the
    // current one is bad enough. set_lane() itself does no collision check at
    // all (it only starts the sideways-drift animation), so nothing else was
    // stopping that. Same idea as entry_slot_free()'s hard check for road
    // handovers, applied to the other place a vehicle's lane can change.
    // Looser than kContactMargin/kIdmS0 on purpose: a lane change is an
    // instant commitment with no gradual IDM convergence to fall back on if
    // the estimate was borderline, so it gets a real margin.
    constexpr double kLaneChangeMinGap = 1.0;

    // ---- junction yielding (M6) ---------------------------------------------
    constexpr double kJunctionZone = 2.0;    // m: conflict radius around the node
    // The distance at which yielding starts is NOT a constant -- it depends on
    // how big the map's blocks are. See JunctionArbiter::junction_approach_
    // and set_junction_approach(); the ceiling below is what the reference uses
    // on its 240 m downtown blocks and stays the upper bound.
    constexpr double kZoneApproachMax = 26.0;  // m
    constexpr double kZoneHorizon = 4.0;     // s: conflicts farther out are ignored
    constexpr double kZoneTieMargin = 0.3;   // s: near-simultaneous arrival window
    constexpr double kZoneBonusCap = 3.0;    // s of head start impatience may claim
    constexpr double kMovingSpeed = 0.5;     // m/s above which a vehicle is moving
    constexpr double kZoneMinSpeed = 1.5;    // m/s floor when estimating arrival
    constexpr double kIgnoreBlockerAfter = 15.0;  // s stuck -> stop giving way
    constexpr double kStoppedSpeed = 0.5;    // m/s below which a vehicle is stopped

    // ---- gridlock breaking (M7) ---------------------------------------------
    constexpr double kForcedGrant = 8.0;     // s that grant lasts
    constexpr double kStuckRescue = 45.0;    // s stopped -> teleport along the route

    // ---- looking past the end of the current road ----------------------------
    // Distance before the junction at which we start watching the queue on the
    // next road. Braking from the fastest desired speed (~22 m/s) at kIdmB = 3.0
    // takes v^2 / 2b = 81 m, but the junction solver already slows traffic on the
    // approach, so 60 m is enough in practice and keeps the scan cheap.
    constexpr double kCrossRoadLookahead = 60.0;

    // Bumper-to-bumper gap between two vehicles on the same road.
    double gap_between(const Vehicle* back, const Vehicle* front) {
        return front->position_on_road - back->position_on_road -
               0.5 * (front->length_meters() + back->length_meters());
    }
}  // namespace

JunctionArbiter::JunctionArbiter(TrafficGraph*& graph,
                                 std::vector<std::unique_ptr<Vehicle>>& vehicles,
                                 TrafficController& controller)
    : graph_(graph), vehicles_(vehicles), controller_(controller) {}

void JunctionArbiter::set_junction_approach(double metres) {
    junction_approach_ = std::clamp(metres, 3.0, 60.0);
}

void JunctionArbiter::reset_counters() {
    lane_changes_ = yields_ = box_waits_ = forced_throughs_ = rescues_ = 0;
}

// Nearest vehicle ahead of `v` in the same lane, looking past the end of the
// current road when nothing blocks us on it.
// Returns {leader, gap}; leader is null when the road ahead is clear.
std::pair<Vehicle*, double> JunctionArbiter::lead_in_lane(Vehicle* v) const {
    Vehicle* best = nullptr;
    double best_gap = 1e9;
    if (!v->current_road) return {nullptr, 1e9};
    for (Vehicle* o : v->current_road->vehicles) {
        if (o == v || o->lane != v->lane) continue;
        if (o->position_on_road <= v->position_on_road) continue;
        const double g = gap_between(v, o);
        if (g < best_gap) { best_gap = g; best = o; }
    }
    if (best) return {best, std::max(0.0, best_gap)};

    // Road ahead is clear, so look into the next road of the route. Without
    // this a vehicle never sees a queue that has spilled back through the
    // junction: Vehicle::advance() hands it over at position ~0 and drops it
    // on top of whoever is already waiting there.
    const double to_end = v->current_road->length - v->position_on_road;
    if (to_end > kCrossRoadLookahead) return {nullptr, 1e9};

    const std::vector<Road*>& route = v->get_route();
    const int next_index = v->get_route_index() + 1;
    if (next_index <= 0 || next_index >= static_cast<int>(route.size()))
        return {nullptr, 1e9};
    Road* next = route[next_index];
    if (!next) return {nullptr, 1e9};

    // advance() remaps the lane on handover, so watch the lane we will land in.
    int lane = v->lane;
    const std::vector<int> allowed = v->allowed_lanes(next->get_lanes());
    if (std::find(allowed.begin(), allowed.end(), lane) == allowed.end())
        lane = allowed.empty() ? 0 : allowed.front();

    for (Vehicle* o : next->vehicles) {
        if (o == v || o->lane != lane) continue;
        // Gap spans the junction: distance left on this road, plus how far the
        // leader has travelled into the next one, minus its length.
        const double g = to_end + o->position_on_road - o->length_meters();
        if (g < best_gap) { best_gap = g; best = o; }
    }
    return {best, std::max(0.0, best_gap)};
}

// Computes the projected IDM acceleration for vehicle v if it were in `lane`.
// Used by the MOBIL model to evaluate lane-changing advantages.
double JunctionArbiter::accel_in_lane(Vehicle* v, int lane) const {
    Road* road = v->current_road;
    if (!road) return 0.0;
    Vehicle* leader = nullptr;
    double gap = 1e9;
    for (Vehicle* o : road->vehicles) {
        if (o == v || o->lane != lane) continue;
        if (o->position_on_road <= v->position_on_road) continue;
        const double g = gap_between(v, o);
        if (g < gap) { gap = g; leader = o; }
    }
    return Vehicle::idm_accel(v->current_speed, v->desired_speed_on(*road),
                              leader ? leader->current_speed : 0.0,
                              leader ? std::max(0.0, gap) : 1e9);
}

// The vehicle immediately behind `v` in `lane` (MOBIL's "new follower").
Vehicle* JunctionArbiter::follower_in_lane(Vehicle* v, int lane) const {
    Vehicle* best = nullptr;
    double best_gap = 1e9;
    if (!v->current_road) return nullptr;
    for (Vehicle* o : v->current_road->vehicles) {
        if (o == v || o->lane != lane) continue;
        if (o->position_on_road >= v->position_on_road) continue;
        const double g = gap_between(o, v);
        if (g < best_gap) { best_gap = g; best = o; }
    }
    return best;
}

// M3/M4: MOBIL lane changes
void JunctionArbiter::solve_lane_changes(double sim_time) {
    for (auto& vp : vehicles_) {
        Vehicle* v = vp.get();
        Road* road = v->current_road;
        if (!road || road->get_lanes() < 2) continue;
        if (!v->lane_change_ready(sim_time)) continue;

        const std::vector<int> allowed = v->allowed_lanes(road->get_lanes());
        if (allowed.size() < 2) continue;

        const double a_cur = accel_in_lane(v, v->lane);
        const double a_free = Vehicle::idm_accel(v->current_speed,
                                                 v->desired_speed_on(*road), 0.0, 1e9);
        // Not actually held up? Then there is nothing to gain by weaving.
        if (v->current_speed >= 1.0 && a_cur >= a_free - kMobilObstruction) continue;

        int best_lane = v->lane;
        double best_gain = kMobilThreshold;

        for (int lane : allowed) {
            if (lane == v->lane || std::abs(lane - v->lane) != 1) continue;

            // Hard gate first, before any gain math: is the slot physically
            // free? Distance is ABSOLUTE, deliberately -- accel_in_lane()
            // only looks at vehicles strictly ahead and follower_in_lane()
            // only at ones strictly behind, so a vehicle sitting at the SAME
            // position in the target lane (two cars side by side, which is
            // exactly what a queue at a stop line produces) is invisible to
            // both, and MOBIL reads the lane as empty. That blind spot is
            // what produced observed overlaps of a full car length, with
            // both vehicles reporting an identical position_on_road.
            bool slot_taken = false;
            for (Vehicle* o : road->vehicles) {
                if (o == v || o->lane != lane) continue;
                const double d = std::abs(o->position_on_road - v->position_on_road);
                const double needed =
                    0.5 * (o->length_meters() + v->length_meters()) + kLaneChangeMinGap;
                if (d < needed) { slot_taken = true; break; }
            }
            if (slot_taken) continue;

            // Kept for the queued-vehicle incentive below: the gap to whoever
            // is ahead in the target lane, once the slot itself is known free.
            double target_gap = 1e9;
            for (Vehicle* o : road->vehicles) {
                if (o == v || o->lane != lane) continue;
                if (o->position_on_road <= v->position_on_road) continue;
                target_gap = std::min(target_gap, gap_between(v, o));
            }

            const double a_new = accel_in_lane(v, lane);
            double gain = a_new - a_cur;

            // Extra incentive for queued/stationary vehicles to escape blocked lanes if target lane is clear ahead
            if (v->current_speed < 1.0) {
                double cur_gap = 1e9;
                for (Vehicle* o : road->vehicles) {
                    if (o == v || o->lane != v->lane) continue;
                    if (o->position_on_road <= v->position_on_road) continue;
                    cur_gap = std::min(cur_gap, gap_between(v, o));
                }
                if (target_gap > cur_gap + 3.0) gain += 1.5;
            }

            // Politeness: subtract the disadvantage we would inflict on the
            // vehicle that ends up behind us. politeness 0 ignores it entirely.
            Vehicle* follower = follower_in_lane(v, lane);
            if (follower) {
                // No hard gap check here: the slot_taken gate above already
                // covers behind as well as ahead. This stays purely MOBIL's
                // own comfort/politeness judgement.
                const double follow_gap = gap_between(follower, v);
                const double f_before = accel_in_lane(follower, lane);
                const double f_after = Vehicle::idm_accel(
                    follower->current_speed, follower->desired_speed_on(*road),
                    v->current_speed, std::max(0.0, follow_gap));
                if (f_after < kMobilSafeDecel) continue;      // unsafe: skip
                gain -= v->lane_politeness() * (f_before - f_after);
            }

            // Flip-flopping back into the lane we just left needs a real win.
            double threshold = kMobilThreshold;
            if (lane == v->prev_lane() && sim_time < v->prev_lane_blocked_until()) {
                threshold = kMobilReturnThreshold;
            }
            if (gain > best_gain && gain > threshold) { best_gain = gain; best_lane = lane; }
        }

        if (best_lane != v->lane) {
            v->set_lane(best_lane, sim_time);
            lane_changes_++;
        }
    }
}

// ---- M6: junction crossing by time-gap acceptance -------------------------
//
// The old rule was "everyone within kZoneApproach of a node forms a group, one
// of them wins, the rest stop". On a synthetic map with ~240 m blocks that is
// almost right, because being within 26 m of a junction is a rare, meaningful
// state. On a real OSM extract the median block is 26.8 m, so a vehicle is
// within 26 m of SOME junction essentially always: 78% of the fleet was
// permanently in a group, N-1 of them stopped every tick, and they stopped for
// each other even when their paths never crossed.
//
// A driver does not obey a per-junction referee. A driver looks at whether the
// box is taken, and at whether anyone else will get there first:
//
//   (a) the box is occupied by a vehicle off a DIFFERENT street, or
//   (b) cross traffic is moving and will arrive clearly earlier than us.
//
// Two vehicles on the same physical street (a road and its "_r" twin) never
// conflict, and neither does a vehicle already on the road we are turning
// into. Arrival is compared in TIME, not distance, and anything farther out
// than kZoneHorizon is ignored entirely.
void JunctionArbiter::solve_junctions(
    double dt, double sim_time,
    std::vector<std::pair<Vehicle*, double>>& leaders,
    std::vector<int>& waiting_on) {

    auto is_crossing = [](const Intersection* n) {
        return n && !n->is_roundabout && n->zone != "waypoint";
    };
    auto both_circulating = [](const Road* a, const Road* b) {
        return a && b && a->is_internal && b->is_internal;
    };
    auto circulating_has_priority = [](const Road* mine, const Road* theirs) {
        return mine && theirs && mine->is_internal && !theirs->is_internal;
    };

    // ---- pre-pass: who is IN each box, and who is approaching it ----------
    const std::size_t num_nodes = graph_ ? graph_->intersections.size() : 0;
    if (node_occupants_.size() < num_nodes) {
        node_occupants_.resize(num_nodes);
        node_approaching_.resize(num_nodes);
    }
    for (std::size_t n = 0; n < num_nodes; ++n) {
        node_occupants_[n].clear();
        node_approaching_[n].clear();
    }

    for (std::size_t i = 0; i < vehicles_.size(); ++i) {
        const Vehicle* w = vehicles_[i].get();
        Road* r = w->current_road;
        if (!r) continue;
        const double to_end = r->length - w->position_on_road;
        const bool moving = w->current_speed > kMovingSpeed;

        // Bounds-checked defensively: index/base_index are only ever as fresh as
        // the last TrafficGraph::reindex(), and an out-of-range index here would
        // otherwise be a silent out-of-bounds write instead of a loud one.
        if (is_crossing(r->target) && r->target->index < node_occupants_.size()) {
            const uint32_t nidx = r->target->index;
            if (to_end < kJunctionZone && (moving || to_end < kJunctionZone - 2.0)) {
                node_occupants_[nidx].push_back({i, r});
            } else if (to_end < junction_approach_) {
                node_approaching_[nidx].push_back(i);
            }
        }
        if (is_crossing(r->source) && w->position_on_road < kJunctionZone &&
            (moving || w->position_on_road < kJunctionZone - 2.0) &&
            r->source->index < node_occupants_.size()) {
            const uint32_t nidx = r->source->index;
            node_occupants_[nidx].push_back({i, r});
        }
    }

    // ---- decide, one vehicle at a time ------------------------------------
    for (std::size_t i = 0; i < vehicles_.size(); ++i) {
        Vehicle* v = vehicles_[i].get();
        Road* road = v->current_road;
        if (!road) continue;
        Intersection* node = road->target;
        if (!is_crossing(node)) continue;
        if (v->ignores_lights() || v->is_forced(sim_time)) continue;
        if (v->stuck_time > kIgnoreBlockerAfter) continue;

        const bool signalized = node->is_signalized() && node->light_system;

        const double to_end = road->length - v->position_on_road;
        if (to_end >= junction_approach_ || to_end <= kJunctionZone) continue;

        const uint32_t my_base_idx = road->base_index;
        const Road* my_next =
            (v->get_route_index() + 1 < static_cast<int>(v->get_route().size()))
                ? v->get_route()[v->get_route_index() + 1]
                : nullptr;
        bool yield = false;
        Vehicle* blocker = nullptr;

        const uint32_t nidx = node->index;

        // (a) the box is taken by someone off another street
        if (!signalized && nidx < node_occupants_.size()) {
            for (const auto& occ : node_occupants_[nidx]) {
                if (occ.vehicle_idx == i) continue;
                if (occ.road->base_index == my_base_idx) continue;  // same street
                if (both_circulating(road, occ.road)) continue;    // same roundabout
                if (circulating_has_priority(road, occ.road)) continue;  // ring beats approach
                if (occ.road == my_next) continue;   // already on the road I want
                yield = true;

                // A wait-for edge only when the occupant is genuinely parked.
                // Waiting behind someone who is driving away is not a deadlock,
                // and recording it as one would make the cycle search fire on
                // ordinary traffic.
                if (vehicles_[occ.vehicle_idx]->current_speed < kStoppedSpeed) {
                    blocker = vehicles_[occ.vehicle_idx].get();
                }
                break;
            }
        }

        // Impatience: every second spent waiting claims up to kZoneBonusCap
        // seconds of head start, so a starved approach eventually takes a slot
        // and the two streams alternate instead of one never moving. This is
        // what keeps the rule from deadlocking into a four-way standoff.
        const double bonus = std::min(v->zone_wait() / 3.0, kZoneBonusCap);

        // (b) moving cross traffic that will clearly get there first
        if (!yield && !signalized && nidx < node_approaching_.size()) {
            const double t_me =
                to_end / std::max(v->current_speed, kZoneMinSpeed) - bonus;
            for (std::size_t j : node_approaching_[nidx]) {
                if (j == i) continue;
                const Vehicle* w = vehicles_[j].get();
                if (w->current_speed < kMovingSpeed) continue;  // parked
                if (!w->current_road) continue;
                if (w->current_road->base_index == my_base_idx) continue;
                if (both_circulating(road, w->current_road)) continue;
                if (circulating_has_priority(road, w->current_road)) continue;  // ring beats approach

                const double d = w->current_road->length - w->position_on_road;
                const double t_w =
                    d / std::max(w->current_speed, kZoneMinSpeed);
                if (t_w > kZoneHorizon) continue;  // far out in time

                // Entering a roundabout gives way to the ring, whoever
                // would arrive first. A time-gap race is right between two
                // ordinary streets, but a roundabout only works because the
                // priority is fixed and everyone knows which way it points.
                if (w->current_road->is_internal && !road->is_internal) {
                    yield = true;
                    break;
                }
                // Ties break by index so the decision is symmetric:
                // exactly one of the pair yields, never both, never
                // neither.
                if (t_w < t_me - kZoneTieMargin ||
                    (std::abs(t_w - t_me) <= kZoneTieMargin && j < i)) {
                    yield = true;
                    break;
                }
            }
        }

        // (c) do not block the box. If the nearest thing downstream is standing
        // still and would halt us before our tail is clear of the junction,
        // wait at the line instead of stalling across it. One car stopped in a
        // crossing blocks every stream through it, not just its own -- this is
        // the rule that costs one vehicle a few seconds and saves the junction.
        // The impatience allowance keeps it from waiting forever behind a queue
        // that is barely creeping.
        bool box_wait = false;
        if (!yield) {
            const double room_needed =
                to_end + kJunctionZone + 1.0 - std::min(v->zone_wait(), 4.0);
            const Vehicle* ahead = leaders[i].first;
            if (leaders[i].second < room_needed && ahead &&
                ahead->current_speed < kStoppedSpeed) {
                yield = box_wait = true;
                blocker = const_cast<Vehicle*>(ahead);
            }
            // Check downstream target exit road (Yellow Box rule): do not enter if exit is jammed
            if (!yield && my_next) {
                const Vehicle* tail_next = my_next->vehicles.empty() ? nullptr : my_next->vehicles.back();
                if (tail_next && tail_next->position_on_road < (v->length_meters() + 3.0) &&
                    tail_next->current_speed < kStoppedSpeed) {
                    yield = box_wait = true;
                    blocker = const_cast<Vehicle*>(tail_next);
                }
            }
        }

        // Emergency evacuation for vehicles trapped inside/at the junction box for > 6.0s
        // Grants temporary priority clearance to drive out of the box instead of wedging indefinitely.
        if (to_end <= kJunctionZone + 4.0 && (v->stuck_time > 6.0 || v->zone_wait() > 6.0)) {
            v->grant_forced_clearance(sim_time + 3.0);
        }

        if (yield) {
            // Brake to the stop line rather than freeze: handing IDM the line
            // as its nearest obstacle is what makes the vehicle slow down for
            // it instead of stopping dead on the spot.
            const double stop_gap =
                to_end - kJunctionZone - v->length_meters() * 0.5;
            if (stop_gap < leaders[i].second) {
                leaders[i] = {nullptr, std::max(0.05, stop_gap)};
                // Giving way to something parked in the box outranks the queue
                // edge: it is the junction, not the car in front, that we are
                // actually stuck on.
                if (blocker) {
                    waiting_on[i] = static_cast<int>(blocker->sim_index);
                }
            }
            v->add_zone_wait(dt);
            (box_wait ? box_waits_ : yields_)++;
        }
    }
}

// M7 stage 1: find deadlocks instead of guessing at them with a stopwatch.
//
// The old rule was "stopped for 10 seconds means deadlocked". That cannot tell
// a queue from a deadlock, and the difference matters: a vehicle 40th in line
// at a busy light is behaving correctly and must be left alone, while four
// vehicles waiting on each other in a ring will never resolve no matter how
// long you wait. Time is the wrong question. The right one is whether the
// wait-for relation has a cycle in it.
//
// Each vehicle waits on at most one other, so the graph is functional: every
// node has out-degree <= 1. That makes cycle detection a single O(n) walk with
// visit stamps -- no recursion, no colours, no union-find.
//
// Walk forward from each unvisited vehicle, stamping as we go. Landing on a
// node stamped in THIS walk means we closed a fresh cycle. Landing on one
// stamped by an earlier walk means we merged into ground already covered, and
// any cycle there was already reported.
std::vector<std::size_t> find_gridlock_cycle_leaders(
    const std::vector<int>& waiting_on, const std::vector<double>& stuck_times) {
    std::vector<std::size_t> leaders;
    const std::size_t n = waiting_on.size();
    std::vector<int> mark(n, 0);
    int stamp = 0;

    for (std::size_t s = 0; s < n; ++s) {
        if (mark[s]) continue;
        ++stamp;
        int u = static_cast<int>(s);
        while (u >= 0 && !mark[u]) {
            mark[u] = stamp;
            u = waiting_on[u];
        }
        if (u >= 0 && mark[u] == stamp) {
            // Break the ring at whoever has suffered longest: one grant is
            // enough, because the vehicle that moves frees the one behind it,
            // and so on around the ring.
            std::size_t best = static_cast<std::size_t>(u);
            for (int w = waiting_on[u]; w != u; w = waiting_on[w]) {
                if (stuck_times[w] > stuck_times[best]) {
                    best = static_cast<std::size_t>(w);
                }
            }
            leaders.push_back(best);
        }
    }
    return leaders;
}

void JunctionArbiter::break_cycles(const std::vector<int>& waiting_on, double sim_time) {
    std::vector<double> stuck(vehicles_.size());
    for (std::size_t i = 0; i < vehicles_.size(); ++i) {
        stuck[i] = vehicles_[i]->stuck_time;
    }

    for (std::size_t leader : find_gridlock_cycle_leaders(waiting_on, stuck)) {
        Vehicle* v = vehicles_[leader].get();
        if (v->is_forced(sim_time)) continue;   // already being pushed through
        v->grant_forced_clearance(sim_time + kForcedGrant);
        forced_throughs_++;
        controller_.log_line("Gridlock broken: " + v->id + " forced through");
    }
}

// M7 stage 2: the vehicle is not in a cycle we can name, and it has been
// standing still for the best part of a minute. Lift it forward along its own
// route. This is the escape hatch of last resort -- if it runs often, the
// traffic model is wrong and this is papering over it.
void JunctionArbiter::solve_gridlock(double sim_time) {
    (void)sim_time;
    for (auto& v : vehicles_) {
        if (v->stuck_time <= kStuckRescue) continue;
        if (v->get_route_index() + 1 >= static_cast<int>(v->get_route().size())) continue;
        // Count only what actually happened. rescue_to() refuses when the slot
        // it would land in is occupied, and deliberately leaves stuck_time alone
        // so the next tick tries again -- so a vehicle that is wedged for good
        // used to book one "rescue" every tick, forever. Measured 19212 of them
        // in 9 sim-seconds, which made the one statistic whose job is to say
        // "the traffic model is wrong" completely unreadable.
        if (v->rescue_to(static_cast<std::size_t>(v->get_route_index()) + 1)) {
            rescues_++;
        }
    }
}
