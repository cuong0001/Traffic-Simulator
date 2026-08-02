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

#include <atomic>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <unordered_map>

#include "core/Road.hpp"
#include "simulation/TrafficController.hpp"
#include "utils/StatisticsManager.hpp"

class TrafficGraph;
class Vehicle;

// Find one vehicle to release from every deadlock cycle in the wait-for graph.
//
// `waiting_on[i]` is the index of the single vehicle i is stuck behind, or -1.
// Out-degree <= 1 makes the graph functional, so one O(n) pass with visit
// stamps finds every cycle without recursion. Returns, per cycle, the member
// that has been stuck longest -- releasing one is enough, because it frees the
// next, and so on around the ring.
//
// Free function on purpose: it is the whole decision, it depends on nothing
// but two arrays, and that makes it the part worth testing directly.
std::vector<std::size_t> find_gridlock_cycle_leaders
    (const std::vector<int>& waiting_on, const std::vector<double>& stuck_times);

class TrafficSimulator {
public:
    // M7: break deadlocks by finding cycles in the wait-for graph. Each entry
    // of `waiting_on` is the index of the one vehicle that vehicle is stuck
    // behind, or -1. A real deadlock is a cycle in that graph.
    void break_cycles(const std::vector<int>& waiting_on);
    double cycle_timer_ = 0.0;   // seconds since the last cycle search

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
    double speed() const { return speed_; }
    void set_auto_spawn(bool on);
    void set_random_events(bool on);

    // traffic-generation tuning
    void set_target_vehicles(int n);   // 0 = unlimited
    void set_spawn_rate(double per_second);

    // ---- the give-way distance -------------------------------------------
    // How close to an unsignalized junction a vehicle has to be before it
    // starts working out whether to give way, in METRES measured from the far
    // end of the road it is on. THIS IS THE KNOB. Outside it a driver simply
    // drives; inside it, it compares arrival times with the cross traffic.
    //
    // Too large and everyone negotiates all the time: with the reference's
    // 26 m on a real OpenStreetMap extract, whose median block is 27 m, a car
    // is inside the window from the moment it turns into a street and half the
    // city is haggling over right of way at any instant. Real drivers commit a
    // second or two out, so the default is derived from the map (see
    // default_junction_approach) and is ~13 m on a city extract.
    //
    // Live-tunable through GET /api/junctionApproach?value=<metres>, and
    // reported in /api/state, so it can be dialled in while watching traffic
    // rather than by rebuilding. Clamped to [7, 60]. Loading a map re-derives
    // it: the right distance belongs to the city, not to the session.
    void set_junction_approach(double metres);
    double junction_approach() const { return junction_approach_; }
    // Routing strategy used by new vehicles: "astar" | "dijkstra" | "bfs".
    bool set_algorithm(const std::string& name);
    // What routes minimise ("distance" | "time"), applied to existing and
    // future vehicles.
    void set_cost_mode(CostMode mode);

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

    // Outcome of an import. Two different things can be true at once -- the
    // map loaded AND it has a problem worth reporting -- so they are separate
    // fields rather than one string that the caller has to interpret.
    struct ImportResult {
        bool ok = false;        ///< did the map actually replace the running one
        std::string message;    ///< failure reason, or a validation warning
    };

    // Adopt a GeoJSON document that arrived over HTTP rather than off disk --
    // an area the user picked out of OpenStreetMap in the browser. Parses into
    // a detached graph first, so a malformed import leaves the running city
    // alone, and only writes the file once the parse succeeded.
    ImportResult import_map(const std::string& name, const std::string& geojson);

    TrafficGraph* graph() const { return graph_; }

    // JSON for the web API. Each locks the simulation internally.
    std::string state_json();  // GET /api/state

    // The published copy of state_json(), refreshed on a background thread and
    // served to HTTP threads WITHOUT taking the simulation mutex. Serialising
    // inside mtx_ means every browser poll blocks a tick, and the work is the
    // same whether one tab asks or six -- so do it once on a schedule and hand
    // out the result. Null until the first snapshot has been built.
    std::shared_ptr<const std::string> state_snapshot();

    // Start the publisher thread. Called once from main().
    void start_publishing();
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
    CostMode cost_mode_ = CostMode::Time;
    // Metres. See set_junction_approach(). Re-derived from the map on load.
    double junction_approach_ = 26.0;

    // per-tick traffic solver (mtx_ held) 
    // A Vehicle only sees its own road, so everything needing a global view
    // lives here: who is ahead, which lane is better, who goes first.
    std::pair<Vehicle*, double> lead_in_lane(Vehicle* v) const;
    double accel_in_lane(Vehicle* v, int lane) const;
    Vehicle* follower_in_lane(Vehicle* v, int lane) const;
    // ---- published state snapshot ------------------------------------------
    void publish_loop();                        ///< publisher thread body
    // A mutex of its OWN. Reusing mtx_ would defeat the entire point: readers
    // would still queue behind the tick they were meant to stop blocking.
    std::mutex snapshot_mtx_;
    std::shared_ptr<const std::string> snapshot_;
    std::atomic<bool> publishing_{false};

    void solve_lane_changes(double sim_time);          // MOBIL

    // Right of way at unsignalized junctions. Takes the leader array that has
    // already been resolved for this tick and, for every vehicle told to give
    // way, replaces its leader with the STOP LINE -- so the vehicle brakes to
    // it under IDM instead of being frozen. Also records the wait-for edge when
    // what it is waiting for is genuinely parked in the box.
    //
    // It needs the leaders as INPUT as well as output: rule (c) below cannot
    // tell whether entering the junction would leave the vehicle stranded
    // across it without knowing what is already in front.
    void solve_junctions(double dt, double sim_time,
                         std::vector<std::pair<Vehicle*, double>>& leaders,
                         std::vector<int>& waiting_on,
                         const std::unordered_map<const Vehicle*, int>& index_of);
    void solve_gridlock(double sim_time);              // forced / rescue

    // Metres derived from the map's own scale -- see set_junction_approach().
    static double default_junction_approach(const TrafficGraph& graph);

    // Solver counters, reported by /api/state for the statistics panel.
    long lane_changes_ = 0;
    // Two different reasons to wait, counted apart because they mean opposite
    // things. yields_ is "I gave way to another stream"; box_waits_ is "the
    // road beyond the junction is full, so I stopped at the line rather than
    // stall across it". A rise in the first says right of way is contested; a
    // rise in the second says the network is backing up.
    long yields_ = 0;
    long box_waits_ = 0;
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
