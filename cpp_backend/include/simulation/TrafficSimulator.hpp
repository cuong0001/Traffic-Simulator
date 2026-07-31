#pragma once
/*
TrafficSimulator.hpp

- The simulation engine. Singleton: exactly one world, shared by the 
tick thread and the HTTP server thread, so every public method locks.
 
- The tick is split into four phases that always run in this order:
    1. events and lane changes   advance signals and events, run MOBIL
    2. leads and physics         resolve every leader, then move everyone
    3. cleanup and spawn         reap arrivals, rescue deadlocks, top up
    4. statistics                sample the network for the charts

- The order matters. A Vehicle can only see its own road, so all the decisions
that need a global view are made in phase 1 and 2 *before* anything moves,
and every leader is snapshotted first -- otherwise the first vehicle to move
would change what the second one sees inside the same tick.
*/

#include <mutex>
#include <random>
#include <string>
#include <utility>
#include <vector>
#include <unordered_map>

#include "simulation/TrafficController.hpp"
#include "utils/StatisticsManager.hpp"

class TrafficGraph;
class Vehicle;

class TrafficSimulator {
public:
    // Singleton access.
    static TrafficSimulator& instance();

    // Load a map and reset the simulation.
    void load(const std::string& map_path);

    // Advance the whole world by dt seconds, scaled by the speed factor.
    void tick(double dt);

    // Spawn a vehicle of `type` routed from intersection `from` to `to`.
    // Returns the new vehicle id, or "" if the pair is invalid, unreachable,
    // or the start of the first road is already occupied.
    std::string spawn_vehicle(const std::string& from, const std::string& to,
                              const std::string& type);

    // Trigger an event ("congestion"|"accident"|"closure"|"light"). An empty
    // target picks a random suitable one; duration <= 0 uses a per-type
    // default. Returns the event id, or "" if it could not be created.
    std::string trigger_event(const std::string& type, const std::string& target,
                              double duration);

    // playback controls
    void set_paused(bool paused);
    void set_speed(double factor);
    void set_auto_spawn(bool on);
    void set_random_events(bool on);

    // traffic-generation tuning 
    void set_target_vehicles(int n);   // 0 = unlimited
    void set_spawn_rate(double per_second);
    // Routing strategy used by new vehicles: "astar" | "dijkstra" | "bfs".
    bool set_algorithm(const std::string& name);

    // vehicle operations
    bool remove_vehicle(const std::string& id);
    int clear_vehicles();                                  // returns how many vehicle on map and delete them
    bool retarget_vehicle(const std::string& id, const std::string& destination);
    bool set_vehicle_algorithm(const std::string& id, const std::string& algo);
    std::string vehicle_detail_json(const std::string& id);

    // live map editing 
    // Each returns the new id, or "" on failure. Vehicles on a removed element
    // are despawned and routed vehicles are asked to replan.
    std::string add_junction_at(double x, double y);
    std::string split_road_near(double x, double y);       // create new junction id
    std::string add_road_between(const std::string& from, const std::string& to,
                                 double speed_kmh, int lanes, bool oneway);
    bool remove_road(const std::string& id);
    bool remove_intersection(const std::string& id);
    bool update_road(const std::string& id, double speed_kmh, bool closed);
    bool set_light(const std::string& node_id, bool on, double green);
    std::string add_building_at(double x, double y, const std::string& kind,
                                const std::string& name, double size);
    bool remove_building(const std::string& name);

    // map files
    std::string maps_json() const;                         // GET /api/maps
    bool save_map(const std::string& name);                // data/maps/<name>.geojson
    bool load_map(const std::string& name);

    TrafficGraph* graph() const { return graph_; }

    // JSON for the web API. Each locks the simulation internally.
    std::string state_json();  // GET /api/state
    std::string map_json();    // GET /api/map

private:
    TrafficSimulator() = default;
    ~TrafficSimulator();
    TrafficSimulator(const TrafficSimulator&) = delete;
    TrafficSimulator& operator=(const TrafficSimulator&) = delete;

    void clear(); // free graph - vehicles - events

    // the four tick phases (mtx_ held) 
    void tick_events_and_lanes(double dt);
    void tick_leads_and_physics(double dt);
    void tick_cleanup_and_spawn(double dt);
    void tick_statistics(double dt);

    void auto_spawn(double dt);  // keep the city busy with random trips
    std::string random_node() const;

    // Non-locking spawn, called from tick()/auto_spawn while mtx_ is held.
    std::string spawn_locked(const std::string& from, const std::string& to,
                             const std::string& type);

    TrafficGraph* graph_ = nullptr;
    StatisticsManager stats_;
    TrafficController controller_;      // owns the events, Subject for vehicles
    std::vector<Vehicle*> vehicles_;

    std::mutex mtx_;
    bool paused_ = false;
    double speed_ = 1.0;
    double sim_time_ = 0.0;
    long tick_count_ = 0;
    int next_id_ = 1;

    bool auto_spawn_ = true;
    double spawn_accum_ = 0.0;      // fractional vehicles owed by spawn_rate_
    double sample_timer_ = 0.0;     // seconds since the last statistics sample
    int target_vehicles_ = 120;     // 0 = unlimited
    double spawn_rate_ = 1.0;       // vehicles per second when auto-spawning
    std::string algorithm_ = "dijkstra";

    // per-tick traffic solver (mtx_ held) 
    // A Vehicle only sees its own road, so everything needing a global view
    // lives here: who is ahead, which lane is better, who goes first.
    std::pair<Vehicle*, double> lead_in_lane(Vehicle* v) const;
    double accel_in_lane(Vehicle* v, int lane) const;
    Vehicle* follower_in_lane(Vehicle* v, int lane) const;
    void solve_lane_changes(double sim_time);          // MOBIL
    void solve_junctions(double dt, double sim_time);  // yielding
    void solve_gridlock(double sim_time);              // forced / rescue

    // Filled by solve_junctions(): for each vehicle told to give way, the
    // bumper-to-stop-line gap it must not drive past this tick. Applied when
    // leaders are resolved, so the vehicle BRAKES to the line under IDM
    // instead of being teleported to a standstill.
    std::unordered_map<Vehicle*, double> zone_stop_gap_;

    // Solver counters, reported by /api/state for the statistics panel.
    long lane_changes_ = 0;
    long yields_ = 0;
    long forced_throughs_ = 0;
    long rescues_ = 0;

    // The following all require mtx_ to be held by the caller.
    void destroy_vehicle_locked(Vehicle* v);          // detach from road and free
    bool remove_road_locked(const std::string& id);   // road + its "_r" twin
    void invalidate_routes_locked();                  // ask everyone to replan
    void reindex_locked();                            // refresh the id caches

    std::string current_map_path_;

    mutable std::mt19937 rng_{2026};
    std::vector<std::string> node_ids_;  // cached ids for random selection
    std::vector<std::string> road_ids_;
};
