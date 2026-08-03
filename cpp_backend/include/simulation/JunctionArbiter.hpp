#pragma once
/*
JunctionArbiter.hpp

- Owns the per-tick traffic solver: MOBIL lane changes, right-of-way at
  unsignalized junctions, and gridlock detection/relief. A Vehicle can only
  see its own road, so everything here needs a global view across the fleet
  -- that is what separates this from Vehicle::advance(), which only needs
  what the arbiter hands it (a leader and a gap).

- Split out of TrafficSimulator so the solver -- the most complex, most
  frequently retuned part of the simulation -- has a class boundary of its
  own, matching the assignment's suggested Simulation-component split.
  TrafficSimulator still drives the tick and owns the vehicles/graph; this
  class is handed references to both rather than owning either, since a
  vehicle's fate (spawn/despawn) is the simulator's call, not the
  arbiter's.
*/

#include <cstddef>
#include <memory>
#include <utility>
#include <vector>

class TrafficGraph;
class TrafficController;
class Vehicle;
class Road;

// Find one vehicle to release from every deadlock cycle in the wait-for
// graph.
//
// `waiting_on[i]` is the index of the single vehicle i is stuck behind, or
// -1. Out-degree <= 1 makes the graph functional, so one O(n) pass with
// visit stamps finds every cycle without recursion. Returns, per cycle, the
// member that has been stuck longest -- releasing one is enough, because it
// frees the next, and so on around the ring.
//
// Free function on purpose: it is the whole decision, it depends on nothing
// but two arrays, and that makes it the part worth testing directly.
std::vector<std::size_t> find_gridlock_cycle_leaders
    (const std::vector<int>& waiting_on, const std::vector<double>& stuck_times);

class JunctionArbiter {
public:
    JunctionArbiter(TrafficGraph*& graph, std::vector<std::unique_ptr<Vehicle>>& vehicles,
                    TrafficController& controller);

    // Nearest vehicle ahead of `v` in its own lane, looking past the end of
    // the current road into the next one when this road is clear. Returns
    // {leader, gap}; leader is null when nothing is ahead.
    std::pair<Vehicle*, double> lead_in_lane(Vehicle* v) const;

    // M3/M4: MOBIL lane-change decisions for the whole fleet.
    void solve_lane_changes(double sim_time);

    // M6: right of way at unsignalized junctions. Takes the leaders already
    // resolved this tick and, for anyone told to give way, replaces their
    // leader with the stop line so they brake to it under IDM. Also records
    // a wait-for edge when what they are waiting for is genuinely parked.
    void solve_junctions(double dt, double sim_time,
                         std::vector<std::pair<Vehicle*, double>>& leaders,
                         std::vector<int>& waiting_on);

    // M7 stage 1: grant forced clearance to one vehicle per deadlock cycle
    // found in the wait-for graph built by solve_junctions().
    void break_cycles(const std::vector<int>& waiting_on, double sim_time);

    // M7 stage 2: last-resort rescue for a vehicle stuck outside any cycle
    // we can name.
    void solve_gridlock(double sim_time);

    // How close to an unsignalized junction a vehicle starts negotiating
    // right of way, in metres. See TrafficSimulator::set_junction_approach
    // for the full rationale; clamped to [3, 60].
    void set_junction_approach(double metres);
    double junction_approach() const { return junction_approach_; }

    // Solver counters, reported by /api/state for the statistics panel.
    long lane_changes() const { return lane_changes_; }
    long yields() const { return yields_; }
    long box_waits() const { return box_waits_; }
    long forced_throughs() const { return forced_throughs_; }
    long rescues() const { return rescues_; }
    void reset_counters();

private:
    double accel_in_lane(Vehicle* v, int lane) const;
    Vehicle* follower_in_lane(Vehicle* v, int lane) const;

    TrafficGraph*& graph_;
    std::vector<std::unique_ptr<Vehicle>>& vehicles_;
    TrafficController& controller_;

    // Metres. See set_junction_approach(). Re-derived from the map on load
    // via TrafficSimulator::default_junction_approach().
    double junction_approach_ = 26.0;

    // Who is IN each junction's box this tick, and who is approaching it.
    // Indexed by Intersection::index; resized lazily in solve_junctions().
    struct JunctionOccupant {
        std::size_t vehicle_idx;
        const Road* road;
    };
    std::vector<std::vector<JunctionOccupant>> node_occupants_;
    std::vector<std::vector<std::size_t>> node_approaching_;

    long lane_changes_ = 0;
    // Two different reasons to wait, counted apart because they mean
    // opposite things. yields_ is "I gave way to another stream"; box_waits_
    // is "the road beyond the junction is full, so I stopped at the line
    // rather than stall across it". A rise in the first says right of way is
    // contested; a rise in the second says the network is backing up.
    long yields_ = 0;
    long box_waits_ = 0;
    long forced_throughs_ = 0;
    long rescues_ = 0;
};
