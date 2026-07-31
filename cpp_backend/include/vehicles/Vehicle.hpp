#pragma once
#include <algorithm>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "routing/PathFinding.hpp"
#include "utils/Observer.hpp"

class Intersection;
class Road;
class Building;
class TrafficGraph;

// Vehicle is an Observer so the event controller can tell it when a road it
// still has to drive is closed, blocked or freed up again.
class Vehicle : public IObserver
{
public:
    std::string id;
    Road* current_road; // The road the vehicle is on
    Intersection* destination; // Destination of vehicle
    double position_on_road; // The distance traveled by the vehicle, measured from the start of the road
    double current_speed; // Current speed of vehicle
    double max_speed; // Max speed of vehicle

    double accel;       // Current acceleration of vehicle
    double max_accel;   // Maximum acceleration of vehicle
    double max_decel;   // Maximum deceleration of vehicle

    std::vector<Road*> route; // List of roads forming the route from the current location to the destination.
    int route_index; // route[route_index]=current_road;
    std::vector<std::string> travel_history; // List of road id the vehicle has passed through
    bool is_reached_destination;

    // --- IDM and lane-change state ---
    double idm_factor{1.0};             // per-vehicle speed personality (0.8 .. 1.0)
    double lateral_offset{0.0};         // current sideways offset from the lane centre
    double target_lateral_offset{0.0};  // where the lane change is heading
    int lane{0};                        // current lane, 0 is the innermost
    double stuck_time{0.0};             // seconds spent standing still
    double last_gap{1e9};               // gap the last advance() actually used

    // Constructor
    Vehicle(std::string id, double max_speed, double max_accel, double max_decel);

    // Destructor
    virtual ~Vehicle() = default;

    // --- core IDM maths ---
    static double idm_accel(double v, double v0, double v_lead, double gap);
    double desired_speed_on(const Road& road) const;

    // --- type hooks overridden by concrete vehicles (polymorphism) ---
    virtual std::string type() const = 0;
    virtual double speed_factor() const { return 1.0; }
    virtual double length_meters() const { return 4.5; }
    virtual double width_meters() const { return 1.8; }
    virtual double congestion_sensitivity() const { return 1.0; }
    virtual bool ignores_lights() const { return false; }
    virtual double lane_politeness() const { return 0.5; }

    virtual std::vector<int> allowed_lanes(int total_lanes) const {
        std::vector<int> lanes;
        for (int i = 0; i < total_lanes; ++i) lanes.push_back(i);
        return lanes;
    }

    // Finding route from node A to node B
    void request_route(TrafficGraph* graph, Intersection* start, Intersection* target);

    // Movement lives in advance(), which the simulator drives. step_by_step()
    // and react_to_congestion() were superseded by it and had no callers left
    // anywhere in src/ or tests/, so they are removed rather than kept in step
    // with a signal model nothing exercised.

    // Refinding route from node C to node B
    void recompute_route(TrafficGraph* graph, Intersection* target);

    // Check if the vehicle has reached its destination.
    bool has_reached_destination() const;

    // Getter
    std::string get_id() const; 
    Road* get_current_road() const; 
    double get_position_on_road() const; 
    const std::vector<std::string>& get_travel_history() const; 
    const std::vector<Road*>& get_route() const; 
    int get_route_index() const; 
    
    double get_current_speed() const; 
    double get_max_speed() const; 

    double get_acceleration() const; 
    double get_max_acceleration() const; 
    double get_max_deceleration() const; 

    int get_lane() const { return lane; }
    double get_lateral_offset() const { return lateral_offset; }

    // Setter
    void set_destination(Intersection* dest) { destination = dest; }

    // ======================================================================
    // Hooks used by TrafficSimulator
    //
    // A Vehicle can only see its own road, so anything needing a global view
    // (the lane next door, the queue on the next road, right of way) is
    // decided by the simulator and handed in through these.
    // ======================================================================

    // --- Observer: an event fired somewhere on the network -----------------
    // Only flags a reroute; the simulator replans on the next tick, because
    // recomputing here would mean editing the route mid-notification.
    void onNotify(const TrafficNotification& notification) override;
    // Force a replan. Used when the graph itself changed (a road was added or
    // removed), which is not one of the TrafficNotification kinds.
    void request_reroute() { needs_reroute_ = true; }
    // Does the part of the route we have NOT driven yet use this road?
    bool remaining_route_contains(const std::string& road_id) const;
    bool consume_reroute_flag();  // read the flag and clear it

    // --- Strategy: which pathfinder this vehicle uses ----------------------
    // The vehicle owns a PathFinding* and calls it through the base class, so
    // swapping the algorithm swaps the object, not a branch in an if-chain.
    // "dijkstra" (default) | "astar" | "bfs"; an unknown name falls back to
    // dijkstra. Set per vehicle at runtime, e.g. from the API.
    void set_algorithm(const std::string& name);
    const std::string& algorithm() const { return algorithm_; }

    // --- simulator-driven movement ----------------------------------------
    // Advance by dt with the leader the simulator found for us. `gap` is the
    // bumper-to-bumper distance, or 1e9 when nothing is ahead. Unlike
    // step_by_step() this does not look for a leader itself, so the simulator
    // can point it at a vehicle in another lane or on the next road.
    void advance(double dt, const Vehicle* lead, double gap, double sim_time);

    // Seconds this vehicle has been travelling (accumulated in advance()).
    double travel_time() const { return travel_time_; }

    // --- multi-lane, driven by the simulator's MOBIL solver ----------------
    int prev_lane() const { return prev_lane_; }
    double prev_lane_blocked_until() const { return prev_lane_blocked_until_; }
    bool lane_change_ready(double sim_time) const;
    void set_lane(int new_lane, double sim_time);

    // --- junction yielding and gridlock relief -----------------------------
    // Impatience: grows while yielding at a junction and resets once the
    // vehicle gets moving, so its gap acceptance becomes progressively bolder.
    // Nobody yields forever, which is what stops one stream starving the other.
    // Capped so a vehicle stuck for minutes does not build an absurd score.
    void add_zone_wait(double seconds) {
        zone_wait_ = std::min(kMaxZoneWait, zone_wait_ + seconds);
    }
    double zone_wait() const { return zone_wait_; }
    bool is_forced(double sim_time) const { return sim_time < forced_until_; }
    void grant_forced_clearance(double until_sim_time) { forced_until_ = until_sim_time; }
    // Last resort: jump to a later road of the route to break a deadlock.
    // Refuses if the target slot is occupied, so the next tick tries again.
    void rescue_to(std::size_t new_index);

private:
    static constexpr double kMaxZoneWait = 15.0;  // s of impatience worth tracking

    // Single entry point to the routing strategy, so request_route() and
    // recompute_route() can never disagree about which one is in use.
    std::vector<Intersection*> run_router(const TrafficGraph& graph,
                                          const std::string& start_id,
                                          const std::string& target_id) const;

    bool needs_reroute_ = false;           // set by onNotify(), read by the simulator
    std::string algorithm_ = "dijkstra";
    std::unique_ptr<PathFinding> router_;  // the Strategy object, swapped at runtime
    double travel_time_ = 0.0;             // accumulated travelling time (s)

    double lane_change_cooldown_ = 0.0;    // s until the next change is allowed
    int prev_lane_ = -1;                   // lane just vacated
    double prev_lane_blocked_until_ = 0.0; // sim time before which going back is discouraged

    double zone_wait_ = 0.0;               // accumulated impatience at a junction
    double forced_until_ = -1.0;           // sim time until which we may push through
};
