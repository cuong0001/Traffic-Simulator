#pragma once
/*
TrafficSimulator.hpp

- The simulation engine. Singleton: exactly one world, shared by the 
tick thread and the HTTP server thread, so every public method locks.
 
- The tick is split into five phases that always run in this order:
    1. events and lane changes   advance signals and events, run MOBIL
    2. leads and physics         resolve every leader, then move everyone
    3. cleanup and spawn         reap arrivals, rescue deadlocks, top up
    4. statistics                sample the network for the charts
    5. history capture           persist one frame every REAL second, for playback

- The order matters. A Vehicle can only see its own road, so all the decisions
that need a global view are made in phase 1 and 2 *before* anything moves,
and every leader is snapshotted first -- otherwise the first vehicle to move
would change what the second one sees inside the same tick.
*/

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <unordered_map>

#include "core/Road.hpp"

// Road-relative, not world-space: this mirrors the per-vehicle fields
// state_json() already sends (road/pos/lane/lat/speed/type), just packed as
// fixed-size binary instead of formatted JSON. The frontend's placeOnRoad()
// remains the only place that turns (road, pos, lane, lat) into world x/y --
// it already accounts for per-road width and the map's cosmetic scale factor
// (roadWOf, US), neither of which the backend knows about. An earlier version
// of this struct computed world x/y here with hardcoded constants; that
// silently disagreed with placeOnRoad() on any map where those constants
// didn't match. See docs/PLAN-fast-ids-and-streaming.md, Phase 3.
#pragma pack(push, 1)
struct VehicleSnapshotBin {
    uint32_t num_id;      // Parsed from the trailing digits of Vehicle::id
    uint32_t base_index;  // Road::base_index of the vehicle's current road --
                           // shared by a two-way street's "id"/"id_r" twin;
                           // bit0 of `flags` says which direction.
    float pos;            // Vehicle::position_on_road (metres)
    float speed;          // Vehicle::current_speed (m/s)
    float lateral_offset; // Vehicle::lateral_offset (metres, in-progress lane change)
    uint8_t lane;         // Vehicle::lane
    uint8_t type;         // 0 car, 1 bus, 2 emergency, 3 motorbike
    uint8_t flags;        // bit0 reverse ("_r") direction, bit1 stopped
};
#pragma pack(pop)
static_assert(sizeof(VehicleSnapshotBin) == 23, "VehicleSnapshotBin layout must be 23 bytes");

#pragma pack(push, 1)
struct BinarySnapshotHeader {
    uint32_t magic;         // 0x54524146 ("TRAF")
    uint16_t version;       // 2: road-relative VehicleSnapshotBin (see above)
    uint16_t vehicle_count;
    double sim_time;
};
#pragma pack(pop)

#include "simulation/HistoryStore.hpp"
#include "simulation/JunctionArbiter.hpp"
#include "simulation/MapEditor.hpp"
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
    double junction_approach() const;
    // Routing strategy used by new vehicles: "astar" | "dijkstra" | "bfs".
    bool set_algorithm(const std::string& name);
    // What routes minimise ("distance" | "time"), applied to existing and
    // future vehicles.
    void set_cost_mode(CostMode mode);

    // live snapshot binary streaming. Locks internally and rebuilds the buffer
    // on the spot -- fine for a direct caller (tests, or a one-off fallback
    // before the first tick has published anything), but the HTTP path should
    // use binary_snapshot() below instead of calling this per request.
    void encode_binary_snapshot(std::vector<uint8_t>& buffer);

    // The published copy of encode_binary_snapshot(), refreshed once per SIM
    // TICK (not on state_snapshot()'s fixed 100 ms timer) and served to HTTP
    // threads without taking the simulation mutex. A VehicleSnapshotBin is a
    // handful of POD field copies -- cheap enough to rebuild every tick,
    // unlike state_json()'s string formatting -- so this tracks the 20 Hz tick
    // rate instead of being throttled to publish_loop()'s cadence. That gap
    // between the two is what made /api/state feel ~50ms stale even after a
    // faster wire format existed. Null until the first tick has run.
    std::shared_ptr<const std::vector<uint8_t>> binary_snapshot();

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

    // Start the publisher thread. Called once from main(). Also loads the
    // history store's on-disk index -- both are "once at process boot" work,
    // and reusing this single already-guarded entry point avoids adding a
    // second one just for that.
    void start_publishing();
    std::string map_json();    // GET /api/map

    // ---- history playback (see HistoryStore.hpp) ---------------------------
    std::string history_range_json();               // GET /api/history/range
    std::string history_state_at_json(double seconds_ago);  // GET /api/history/at

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
    void tick_history_capture(double dt);

    void auto_spawn(double dt);  // keep the city busy with random trips
    std::string random_node() const;

    // Non-locking spawn, called from tick()/auto_spawn while mtx_ is held.
    std::string spawn_locked(const std::string& from, const std::string& to,
                             const std::string& type);

    TrafficGraph* graph_ = nullptr;
    StatisticsManager stats_;
    TrafficController controller_;      // owns the events, Subject for vehicles
    std::vector<std::unique_ptr<Vehicle>> vehicles_;
    int next_id_ = 1;

    // Junction right-of-way, lane changes and gridlock relief, and live map
    // editing: split out of TrafficSimulator so each has a class boundary of
    // its own (see JunctionArbiter.hpp / MapEditor.hpp). Both are handed
    // references to graph_/vehicles_/controller_/next_id_ above rather than
    // owning them -- vehicle lifetime and the graph pointer itself stay
    // TrafficSimulator's call. Declared after what they reference so
    // constructor-order matches member-initializer-list order.
    JunctionArbiter arbiter_{graph_, vehicles_, controller_};
    MapEditor map_editor_{graph_, vehicles_, controller_, next_id_};

    std::mutex mtx_;
    bool paused_ = false;
    double speed_ = 1.0;
    double sim_time_ = 0.0;
    long tick_count_ = 0;

    // Real epoch seconds when load() last ran -- the anchor the history
    // scrubber's calendar clock (calendarTime = this + sim_time_) counts
    // forward from. See HistoryFrame::calendar_time for why it deliberately
    // is NOT just "now": it has to run at sim speed, not real speed.
    double sim_clock_base_epoch_ = 0.0;

    bool auto_spawn_ = true;
    double spawn_accum_ = 0.0;      // fractional vehicles owed by spawn_rate_
    double sample_timer_ = 0.0;     // seconds since the last statistics sample

    // ---- history playback ---------------------------------------------------
    // Rolling 24h window, indexed by REAL capture time rather than sim_time_
    // (which resets on every load()) -- see HistoryStore.hpp for why.
    HistoryStore history_{"data/history", 86400.0};
    double last_history_capture_real_ = 0.0;  // epoch seconds; 0 forces an immediate first capture
    int target_vehicles_ = 120;     // 0 = unlimited
    double spawn_rate_ = 1.0;       // vehicles per second when auto-spawning
    std::string algorithm_ = "dijkstra";
    CostMode cost_mode_ = CostMode::Time;

    // ---- published state snapshot ------------------------------------------
    void publish_loop();                        ///< publisher thread body
    // A mutex of its OWN. Reusing mtx_ would defeat the entire point: readers
    // would still queue behind the tick they were meant to stop blocking.
    std::mutex snapshot_mtx_;
    std::shared_ptr<const std::string> snapshot_;
    std::atomic<bool> publishing_{false};

    // ---- published BINARY snapshot, refreshed from inside tick() ----------
    // Same shared_ptr-swap trick as the JSON snapshot above, with its own
    // mutex for the same reason -- but published every tick (see tick()),
    // not on a timer, because a POD copy is cheap enough to afford that and
    // vehicle positions are the thing that actually needs tick-rate freshness.
    void encode_binary_snapshot_locked(std::vector<uint8_t>& buffer);  // mtx_ already held
    void publish_binary_snapshot_locked();                             // mtx_ already held
    std::mutex bin_snapshot_mtx_;
    std::shared_ptr<const std::vector<uint8_t>> bin_snapshot_;

    // M7: break deadlocks by finding cycles in the wait-for graph built by
    // arbiter_.solve_junctions(). Only ever called from tick_leads_and_physics
    // on its own cadence (cycle_timer_/kCycleCheckPeriod), so it stays a
    // TrafficSimulator-private forwarder rather than public API.
    void break_cycles(const std::vector<int>& waiting_on);
    double cycle_timer_ = 0.0;   // seconds since the last cycle search

    // Metres derived from the map's own scale -- see set_junction_approach().
    static double default_junction_approach(const TrafficGraph& graph);

    // The following all require mtx_ to be held by the caller.
    void destroy_vehicle_locked(Vehicle* v);          // detach from road/events; caller erases from vehicles_ to free

    std::string current_map_path_;

    mutable std::mt19937 rng_{2026};
};
