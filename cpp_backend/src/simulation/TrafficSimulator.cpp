#include "simulation/TrafficSimulator.hpp"

#include <algorithm>
#include <chrono>
#include <thread>
#include <cmath>
#include <cstring>
#include <unordered_map>
#include <filesystem>
#include <sstream>
#include <fstream>
#include <memory>

#include "core/Building.hpp"
#include "core/Graph.hpp"
#include "core/Intersection.hpp"
#include "core/Road.hpp"
#include "core/TrafficLight.hpp"
#include "events/TrafficEvent.hpp"
#include "utils/MapLoader.hpp"
#include "vehicles/Vehicle.hpp"
#include "vehicles/VehicleTypes.hpp"

namespace {
    // Free space a new vehicle needs at the start of its first road, on top of its
    // own length. Spawning is skipped when the slot is taken; auto_spawn simply
    // tries again on a later tick.
    constexpr double kSpawnClearance = 3.0;

    // ---- M7 wait-for graph ----------------------------------------------------
    // Used by tick_leads_and_physics(), which sits above the solver block, so they
    // have to be declared here rather than with the other M6/M7 constants.
    constexpr double kCycleCheckPeriod = 2.0;  // s between wait-for cycle searches
    // m: closer than this to a stopped car and we count as queued behind it.
    // Matches the reference. At 12 m it recorded a wait-for edge for cars that
    // were merely following at a normal distance, which handed the M7 cycle
    // search ordinary queueing to interpret as deadlock.
    constexpr double kWaitEdgeGap = 6.0;
    constexpr double kStoppedSpeed = 0.5;      // m/s below which a vehicle is stopped
}

// Singleton 
TrafficSimulator& TrafficSimulator::instance() {
    static TrafficSimulator sim; 
    return sim;
}

TrafficSimulator::~TrafficSimulator() { clear(); }

void TrafficSimulator::clear() {
    // Detach before the unique_ptrs go out of scope below. The controller
    // keeps raw IObserver* pointers, so a vehicle freed while still
    // subscribed leaves a dangling entry that the next notifyAll() would
    // call into.
    for (auto& v : vehicles_) {
        controller_.detach(v.get());
    }
    vehicles_.clear();  // unique_ptr destructors free each Vehicle
    controller_.clear();
    controller_.set_graph(nullptr);
    delete graph_;
    graph_ = nullptr;
}

void TrafficSimulator::load(const std::string& map_path) 
{
    std::unique_ptr<TrafficGraph> loaded(new TrafficGraph());
    MapLoader::load_from_geojson(*loaded, map_path);
    std::lock_guard<std::mutex> lock(mtx_);
    clear();
    graph_ = new TrafficGraph();
    MapLoader::load_from_geojson(*graph_, map_path);
    current_map_path_ = map_path;
    controller_.set_graph(graph_);

    reindex_locked();

    stats_ = StatisticsManager();
    sim_time_ = 0.0;
    tick_count_ = 0;
    next_id_ = 1;
    spawn_accum_ = 0.0;
    sample_timer_ = 0.0;
    lane_changes_ = yields_ = box_waits_ = forced_throughs_ = rescues_ = 0;
    // Re-derived on every load, deliberately: the right give-way distance is a
    // property of the city you are driving in, not of the session. Carrying a
    // hand-tuned 12 m onto a map with 200 m blocks would have cars deciding
    // almost inside the junction, and nothing on screen would explain why.
    junction_approach_ = default_junction_approach(*graph_);
}

// How close to a junction a driver starts working out whether to give way.
//
// A fixed number cannot be right for every map. 26 m is a final approach on the
// reference's hand-drawn 240 m downtown blocks -- 11% of a block. On a real
// OpenStreetMap extract the median block is about 27 m, so the same 26 m covers
// the WHOLE street: a car is deciding about the junction ahead from the instant
// it turns into the road, never gets a free stretch, and half the city is
// negotiating right of way at any one time. Measured on small_map: 48% of
// segments are shorter than 26 m, and 51% of the fleet sat inside the window
// against the reference's 31% on the same map.
//
// Half a block is the rule: far enough to see the junction coming, close enough
// that it is about this junction and not the next one. Floored so the window
// always clears the box itself, ceilinged at the reference's value.
double TrafficSimulator::default_junction_approach(const TrafficGraph& graph) {
    std::vector<double> lens;
    lens.reserve(graph.edges.size());
    for (const auto& [id, road] : graph.edges) {
        if (road && road->length > 0.0) lens.push_back(road->length);
    }
    if (lens.empty()) return 26.0;
    const std::size_t mid = lens.size() / 2;
    std::nth_element(lens.begin(), lens.begin() + mid, lens.end());
    const double median = lens[mid];
    return std::clamp(median * 0.5, 10.0, 26.0);
}

void TrafficSimulator::set_junction_approach(double metres) {
    std::lock_guard<std::mutex> lock(mtx_);
    junction_approach_ = std::clamp(metres, 3.0, 60.0);
}

// ==========================================================================
// The tick loop, in four phases. See the header for why the order matters.
// ==========================================================================
void TrafficSimulator::tick(double dt) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (paused_ || !graph_) return;
    dt *= speed_;

    tick_events_and_lanes(dt);
    tick_leads_and_physics(dt);
    tick_cleanup_and_spawn(dt);
    tick_statistics(dt);

    // Positions are what actually needs tick-rate freshness; everything else
    // in state_json() (lights, roads, events, stats) still goes through
    // publish_loop()'s own, slower cadence.
    publish_binary_snapshot_locked();
}

// Phase 1: advance the clock, the signals and the events, then let anyone who
// wants to change lane do so. All of this happens before a single vehicle
// moves, so every vehicle in phase 2 sees the same world.
void TrafficSimulator::tick_events_and_lanes(double dt) {
    sim_time_ += dt;
    ++tick_count_;

    // Signals need no tick: a junction's colour is a pure function of
    // sim_time_ and that junction's own phase offset (see TrafficLight).

    // Recompute congestion from the traffic actually on each road. Without
    // this the level only ever moves when an event touches it, so routing
    // never learns which roads are busy and congestion_sensitivity() has
    // nothing to react to.
    for (auto& [id, road] : graph_->edges) {
        if (road) road->update_travel_cost();
    }

    // Events start, run down and undo themselves; vehicles hear about it
    // through the Observer and set their reroute flag.
    controller_.update(dt);

    // Replan anyone an event flagged, capped to a maximum per tick to prevent
    // single-tick latency spikes when many vehicles replan.
    int reroutes_this_tick = 0;
    constexpr int kMaxReroutesPerTick = 15;
    for (auto& v : vehicles_) {
        if (reroutes_this_tick >= kMaxReroutesPerTick) break;
        if (!v->can_recompute() || !v->destination) continue;
        if (!v->consume_reroute_flag()) continue;
        const auto t0 = std::chrono::steady_clock::now();
        v->recompute_route(graph_, v->destination);
        const double micros = std::chrono::duration<double, std::micro>(
                                  std::chrono::steady_clock::now() - t0).count();
        stats_.recordRouteComputation(v->algorithm(), micros, v->last_nodes_explored(), !v->get_route().empty());
        reroutes_this_tick++;
    }

    solve_lane_changes(sim_time_);
}

// Phase 2: work out who is in front of whom, decide right of way, then move
// everyone. Leaders are snapshotted first -- if we resolved them lazily, the
// first vehicle to move would change what the second one sees this tick.
void TrafficSimulator::tick_leads_and_physics(double dt) {
    for (std::size_t i = 0; i < vehicles_.size(); ++i) {
        vehicles_[i]->sim_index = i;
    }

    std::vector<std::pair<Vehicle*, double>> leaders;
    leaders.reserve(vehicles_.size());
    for (auto& v : vehicles_) leaders.push_back(lead_in_lane(v.get()));

    std::vector<int> waiting_on(vehicles_.size(), -1);

    for (std::size_t i = 0; i < vehicles_.size(); ++i) {
        // Queued behind a car that is not moving: a wait-for edge.
        Vehicle* lead = leaders[i].first;
        if (lead && leaders[i].second < kWaitEdgeGap &&
            lead->current_speed < kStoppedSpeed) {
            waiting_on[i] = static_cast<int>(lead->sim_index);
        }
    }

    solve_junctions(dt, sim_time_, leaders, waiting_on);
    solve_gridlock(sim_time_);

    // Searching every tick would be wasted work: a deadlock that exists now
    // still exists in two seconds, and the search is O(n) each time.
    cycle_timer_ += dt;
    if (cycle_timer_ >= kCycleCheckPeriod) {
        cycle_timer_ = 0.0;
        break_cycles(waiting_on);
    }
    
    for (std::size_t i = 0; i < vehicles_.size(); ++i) {
        vehicles_[i]->advance(dt, leaders[i].first, leaders[i].second, sim_time_);
    }
}

// Phase 3: retire the vehicles that arrived and top the city back up.
void TrafficSimulator::tick_cleanup_and_spawn(double dt) {
    for (auto it = vehicles_.begin(); it != vehicles_.end();) {
        Vehicle* v = it->get();
        if (v->has_reached_destination()) {
            double length = 0.0;
            for (Road* r : v->get_route()) length += r->length;
            stats_.recordTripCompleted(v->travel_time(), length, v->type());
            controller_.detach(v);
            it = vehicles_.erase(it);  // unique_ptr destructor frees the Vehicle
        } else {
            ++it;
        }
    }

    if (auto_spawn_) auto_spawn(dt);
}

// Phase 4: sample the network for the congestion chart. Every couple of
// seconds, not every tick -- 60 samples a second would be noise and would
// swamp the history buffer in under a minute.
void TrafficSimulator::tick_statistics(double dt) {
    sample_timer_ += dt;
    if (sample_timer_ < 2.0) return;
    sample_timer_ = 0.0;

    double total = 0.0;
    for (const auto& [id, road] : graph_->edges) {
        if (road) total += road->get_congestion_level();
    }
    const double avg = graph_->edges.empty()
                     ? 0.0
                     : total / static_cast<double>(graph_->edges.size());
    stats_.pushSample(sim_time_, avg, static_cast<int>(vehicles_.size()));
}

// spawning (Factory)
std::string TrafficSimulator::spawn_vehicle(const std::string& from, const std::string& to,
                                            const std::string& type) {
    std::lock_guard<std::mutex> lock(mtx_);
    return spawn_locked(from, to, type);
}

std::string TrafficSimulator::spawn_locked(const std::string& from, const std::string& to,
                                           const std::string& type) {
    if (!graph_) return "";
    auto fit = graph_->intersections.find(from);
    auto tit = graph_->intersections.find(to);
    if (fit == graph_->intersections.end() || tit == graph_->intersections.end()) return "";
    if (from == to) return "";

    // The id's type prefix must match what type() reports afterward -- the
    // binary snapshot (encode_binary_snapshot) reconstructs a display id on
    // the frontend as TYPE_NAMES[type] + num_id, which only works if the
    // prefix is always one of the four canonical strings. `type` is whatever
    // the caller passed (e.g. a free-form /api/spawn query param); canonicalize
    // it the same way create() resolves an unrecognized type to "car".
    const std::string id = VehicleFactory::canonical_type(type) + std::to_string(next_id_++);
    // Owned locally until the very end: an early "unreachable" or
    // "no lane" return just lets `owned` go out of scope and free it, rather
    // than a manual delete on every such path. Only a successful spawn moves
    // it into vehicles_, which is what owns it from then on.
    std::unique_ptr<Vehicle> owned = VehicleFactory::create(type, id);
    Vehicle* v = owned.get();
    v->set_algorithm(algorithm_);
    const auto t0 = std::chrono::steady_clock::now();
    v->request_route(graph_, fit->second.get(), tit->second.get());
    const double micros = std::chrono::duration<double, std::micro>(
                              std::chrono::steady_clock::now() - t0).count();
    stats_.recordRouteComputation(algorithm_, micros, v->last_nodes_explored(), !v->get_route().empty());
    if (v->get_route().empty()) {  // unreachable
        return "";
    }
    // request_route() already put v on its first road. Pick the best allowed lane
    // with the clearest entrance gap to distribute traffic across available lanes.
    if (Road* first = v->current_road) {
        const std::vector<int> ok = v->allowed_lanes(first->lanes);
        int best_lane = v->lane;
        double best_gap = -1.0;
        const double needed = v->length_meters() + kSpawnClearance;

        for (int l : ok) {
            bool slot_ok = true;
            double min_pos = 1e9;
            for (Vehicle* o : first->vehicles) {
                if (o == v || o->lane != l) continue;
                if (o->position_on_road < needed) {
                    slot_ok = false;
                    break;
                }
                if (o->position_on_road < min_pos) min_pos = o->position_on_road;
            }
            if (slot_ok && min_pos > best_gap) {
                best_gap = min_pos;
                best_lane = l;
            }
        }

        if (best_gap < 0.0) {  // All allowed lanes blocked at entrance
            first->remove_vehicle(v);
            return "";
        }

        v->lane = best_lane;
    }
    controller_.attach(v);  // Observer: v now hears about events
    vehicles_.push_back(std::move(owned));
    return id;
}

// events
// Events live in the controller: it owns them, applies them, and broadcasts
// the notifications the events themselves describe.
std::string TrafficSimulator::trigger_event(const std::string& type, const std::string& target,
                                            double duration) {
    std::lock_guard<std::mutex> lock(mtx_);
    return controller_.trigger_event(type, target, duration);
}

// playback controls
void TrafficSimulator::set_paused(bool paused) {
    std::lock_guard<std::mutex> lock(mtx_);
    paused_ = paused;
}
void TrafficSimulator::set_speed(double factor) {
    std::lock_guard<std::mutex> lock(mtx_);
    speed_ = std::max(0.1, std::min(20.0, factor));
}
void TrafficSimulator::set_auto_spawn(bool on) {
    std::lock_guard<std::mutex> lock(mtx_);
    auto_spawn_ = on;
}
void TrafficSimulator::set_random_events(bool on) {
    std::lock_guard<std::mutex> lock(mtx_);
    controller_.set_random_events(on);
}

// random helpers (called with mtx_ held)
std::string TrafficSimulator::random_node() const {
    if (node_ids_.empty()) return "";
    std::uniform_int_distribution<std::size_t> dist(0, node_ids_.size() - 1);
    return node_ids_[dist(rng_)];
}

void TrafficSimulator::auto_spawn(double dt) {
    // target_vehicles_ == 0 means "unlimited": keep spawning at the rate.
    const bool unlimited = target_vehicles_ <= 0;
    if (spawn_rate_ <= 0.0 ||
        (!unlimited && static_cast<int>(vehicles_.size()) >= target_vehicles_)) {
        spawn_accum_ = 0.0;
        return;
    }

    // Accumulate fractional vehicles so a rate of 0.3/s spawns one every ~3 s
    // instead of rounding to nothing, and a rate of 40/s can spawn several in
    // one tick.
    spawn_accum_ += spawn_rate_ * dt;
    int due = static_cast<int>(spawn_accum_);
    if (due <= 0) return;
    if (!unlimited) {
        due = std::min(due, target_vehicles_ - static_cast<int>(vehicles_.size()));
    }
    if (due <= 0) return;
    spawn_accum_ -= due;

    for (int i = 0; i < due; ++i) {
        // Weighted type: mostly cars, some buses, rare emergencies.
        std::uniform_int_distribution<int> roll(0, 99);
        const int r = roll(rng_);
        const std::string type = r < 78 ? "car" : r < 94 ? "bus" : "emergency";

        // A trip can fail (unreachable pair, kerb still occupied); try a few
        // other pairs before giving this one up.
        for (int attempt = 0; attempt < 6; ++attempt) {
            const std::string from = random_node();
            const std::string to = random_node();
            if (from != to && !spawn_locked(from, to, type).empty()) break;
        }
    }
}

namespace {
    // JSON for the web API
    std::string jstr(const std::string& s) {
        std::string out = "\"";
        for (char c : s) {
            if (c == '"' || c == '\\') out += '\\';
            out += c;
        }
        out += '"';
        return out;
    }
}

// ---------------------------------------------------------------------------
// Published state snapshot.
//
// /api/state used to serialise the whole world while holding mtx_, so every
// poll from the browser stalled a simulation tick -- and the UI polls twice a
// second, per open tab. The work is identical whether one tab asks or six, so
// do it once on a schedule and hand out the result.
//
// The snapshot is a shared_ptr<const string>: readers take a copy of the
// pointer under a tiny mutex and then read the string with no lock at all,
// because nothing can mutate a string that is already published. The publisher
// builds the next one separately and swaps it in.
// ---------------------------------------------------------------------------
void TrafficSimulator::start_publishing() {
    if (publishing_.exchange(true)) return;   // already running
    // Detached, like the tick thread in main(): the simulator is a singleton
    // that lives as long as the process, so there is no shutdown to join at.
    std::thread(&TrafficSimulator::publish_loop, this).detach();
}

void TrafficSimulator::publish_loop() {
    while (publishing_.load()) {
        auto fresh = std::make_shared<const std::string>(state_json());
        {
            std::lock_guard<std::mutex> lock(snapshot_mtx_);
            snapshot_ = std::move(fresh);
        }
        // 10 Hz: faster than the UI polls (2-4 Hz), reducing mutex lock contention
        // with the simulation thread while keeping frontend updates fluid.
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

std::shared_ptr<const std::string> TrafficSimulator::state_snapshot() {
    std::lock_guard<std::mutex> lock(snapshot_mtx_);
    return snapshot_;
}

std::string TrafficSimulator::state_json() {
    std::lock_guard<std::mutex> lock(mtx_);
    std::ostringstream os;
    os << "{";
    os << "\"time\":" << sim_time_ << ",";
    os << "\"paused\":" << (paused_ ? "true" : "false") << ",";
    os << "\"speed\":" << speed_ << ",";
    // The control panel shows these. Without them in the state the inputs can
    // only display whatever was hard-coded in index.html, which drifts from
    // what the simulator is actually doing the moment either side changes.
    os << "\"targetVehicles\":" << target_vehicles_ << ",";
    os << "\"spawnRate\":" << spawn_rate_ << ",";
    os << "\"junctionApproach\":" << junction_approach_ << ",";
    os << "\"algorithm\":" << jstr(algorithm_) << ",";
    os << "\"autoSpawn\":" << (auto_spawn_ ? "true" : "false") << ",";
    os << "\"randomEvents\":" << (controller_.random_events() ? "true" : "false") << ",";

    // vehicles
    os << "\"vehicles\":[";
    bool first = true;
    for (const auto& v : vehicles_) {
        if (!v->current_road) continue;
        if (!first) os << ",";
        first = false;
        os << "{\"id\":" << jstr(v->id)
           << ",\"road\":" << jstr(v->current_road->id)
           << ",\"pos\":" << v->position_on_road
           << ",\"speed\":" << v->current_speed
           << ",\"lane\":" << v->lane
           << ",\"lat\":" << v->lateral_offset
           << ",\"recalcs\":" << v->recalculations()
           << ",\"state\":" << static_cast<int>(v->state())
           << ",\"type\":" << jstr(v->type()) << "}";
    }
    os << "],";

    // signalized lights
    // One entry per APPROACH, not per junction. A junction shows different
    // colours to different approaches, so an entry keyed by junction id cannot
    // describe it -- the frontend has to know which way is actually stopped.
    // "n" is the junction id, so heads can still be grouped by node.
    os << "\"lights\":[";
    first = true;
    if (graph_) {
        for (const auto& [id, road] : graph_->edges) {
            if (road->signal_group < 0 || !road->target) continue;
            if (!road->target->is_signalized) continue;
            if (!first) os << ",";
            first = false;
            os << "{\"id\":" << jstr(id)
               << ",\"n\":" << jstr(road->target->name)
               << ",\"g\":" << road->signal_group
               << ",\"d\":" << road->signal_stop_from_end
               << ",\"state\":"
               << road->target->signal_state_for(road->signal_group, sim_time_) << "}";
        }
    }
    os << "],";

    // congested / closed roads only (keep the payload small)
    os << "\"roads\":[";
    first = true;
    if (graph_) {
        for (const auto& [id, road] : graph_->edges) {
            if (road->congestion_level <= 0.01 && !road->is_closed) continue;
            if (!first) os << ",";
            first = false;
            os << "{\"id\":" << jstr(id)
               << ",\"congestion\":" << road->congestion_level
               << ",\"closed\":" << (road->is_closed ? "true" : "false") << "}";
        }
    }
    os << "],";

    // active events
    os << "\"events\":[";
    first = true;
    for (const auto& e : controller_.events()) {
        if (!first) os << ",";
        first = false;
        os << "{\"id\":" << jstr(e->id()) << ",\"type\":" << jstr(e->type())
           << ",\"target\":" << jstr(e->target())
           << ",\"remaining\":" << e->remaining()
           << ",\"description\":" << jstr(e->description()) << "}";
    }
    os << "],";

    // rolling event log for the UI panel
    os << "\"log\":[";
    first = true;
    for (const std::string& line : controller_.log()) {
        if (!first) os << ",";
        first = false;
        os << jstr(line);
    }
    os << "],";

    os << "\"solver\":{\"lane_changes\":" << lane_changes_
       << ",\"yields\":" << yields_
       << ",\"box_waits\":" << box_waits_
       << ",\"forced\":" << forced_throughs_
       << ",\"rescues\":" << rescues_
       << ",\"ticks\":" << tick_count_ << "},";
    // Live counts the simulator owns. Everything aggregated over the run lives
    // in StatisticsManager instead, under "stats".
    os << "\"active\":" << vehicles_.size() << ",";
    os << "\"spawned\":" << (next_id_ - 1) << ",";
    os << "\"stats\":" << stats_.toJson(true);
    os << "}";
    return os.str();
}

std::string TrafficSimulator::map_json() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!graph_) return "{}";
    return MapLoader::map_json_string(*graph_);
}

// ==========================================================================
// Tuning, vehicle operations and live map editing (web API)
// ==========================================================================

void TrafficSimulator::reindex_locked() {
    node_ids_.clear();
    road_ids_.clear();
    if (!graph_) return;
    for (const auto& [id, node] : graph_->intersections) node_ids_.push_back(id);
    for (const auto& [id, road] : graph_->edges) road_ids_.push_back(id);
    // Live map edits (add/split/remove a junction or road) create or drop
    // Intersection/Road objects after the loader's initial reindex(). Without
    // this, a newly created node/road keeps its default index 0 and silently
    // aliases whatever already owns that slot in solve_junctions()'s per-node
    // buckets.
    graph_->reindex();
}

namespace {
// Must match the frontend's TYPE_NAMES table exactly -- the frontend
// reconstructs a vehicle's display id as TYPE_NAMES[type] + num_id.
uint8_t binary_type_code(const std::string& vtype) {
    if (vtype == "bus") return 1;
    if (vtype == "emergency") return 2;
    if (vtype == "motorbike") return 3;
    return 0;  // car, and any unrecognized type
}
}  // namespace

void TrafficSimulator::encode_binary_snapshot(std::vector<uint8_t>& buffer) {
    std::lock_guard<std::mutex> lock(mtx_);
    encode_binary_snapshot_locked(buffer);
}

void TrafficSimulator::encode_binary_snapshot_locked(std::vector<uint8_t>& buffer) {
    const uint16_t count = static_cast<uint16_t>(vehicles_.size());
    const std::size_t payload_bytes = sizeof(BinarySnapshotHeader) + count * sizeof(VehicleSnapshotBin);
    if (buffer.size() < payload_bytes) buffer.resize(payload_bytes);

    BinarySnapshotHeader header;
    header.magic = 0x54524146; // "TRAF"
    header.version = 2;
    header.vehicle_count = count;
    header.sim_time = sim_time_;
    std::memcpy(buffer.data(), &header, sizeof(header));

    VehicleSnapshotBin* dest = reinterpret_cast<VehicleSnapshotBin*>(buffer.data() + sizeof(header));
    for (std::size_t i = 0; i < count; ++i) {
        const Vehicle* v = vehicles_[i].get();
        VehicleSnapshotBin& out = dest[i];
        out.num_id = v->num_id;
        out.type = binary_type_code(v->type());

        const Road* r = v->current_road;
        out.base_index = r ? r->base_index : 0;
        out.pos = static_cast<float>(v->position_on_road);
        out.speed = static_cast<float>(v->current_speed);
        out.lateral_offset = static_cast<float>(v->lateral_offset);
        out.lane = static_cast<uint8_t>(std::clamp(v->lane, 0, 255));

        uint8_t flags = 0;
        if (r && r->reverse_twin) flags |= 1;
        if (v->current_speed < 0.5) flags |= 2;
        out.flags = flags;
    }
}

void TrafficSimulator::publish_binary_snapshot_locked() {
    auto buf = std::make_shared<std::vector<uint8_t>>();
    encode_binary_snapshot_locked(*buf);
    std::lock_guard<std::mutex> lock(bin_snapshot_mtx_);
    bin_snapshot_ = std::move(buf);
}

std::shared_ptr<const std::vector<uint8_t>> TrafficSimulator::binary_snapshot() {
    std::lock_guard<std::mutex> lock(bin_snapshot_mtx_);
    return bin_snapshot_;
}

// Detaches a vehicle from its road and the event Subject, but does not free
// it: vehicles_ owns it via unique_ptr, so the caller frees it by erasing
// the corresponding entry from vehicles_ right after calling this.
void TrafficSimulator::destroy_vehicle_locked(Vehicle* v) {
    if (!v) return;
    if (v->current_road) v->current_road->remove_vehicle(v);
    controller_.detach(v);
}

void TrafficSimulator::invalidate_routes_locked() {
    // The graph itself changed under their wheels. This is not one of the
    // TrafficNotification kinds (those describe events on an existing network),
    // so ask each vehicle directly instead of inventing a Kind for it.
    for (auto& v : vehicles_) v->request_reroute();
}

// Drop a road and its reverse twin, detaching any vehicles on them.
// Caller holds mtx_ and is responsible for reindex/invalidate.
bool TrafficSimulator::remove_road_locked(const std::string& id) {
    if (!graph_) return false;
    bool removed = false;
    for (const std::string& rid : {id, id + "_r"}) {
        auto it = graph_->edges.find(rid);
        if (it == graph_->edges.end()) continue;
        Road* road = it->second.get();

        // Vehicles standing on a deleted road have nowhere to be: despawn them.
        for (auto vit = vehicles_.begin(); vit != vehicles_.end();) {
            if ((*vit)->current_road == road) {
                (*vit)->current_road = nullptr;   // already dying with the road
                destroy_vehicle_locked(vit->get());
                vit = vehicles_.erase(vit);
            } else {
                ++vit;
            }
        }
        if (road->source) {
            auto& out = road->source->outgoing_edges;
            out.erase(std::remove(out.begin(), out.end(), road), out.end());
        }
        if (road->target) {
            auto& in = road->target->incoming_edges;
            in.erase(std::remove(in.begin(), in.end(), road), in.end());
        }
        graph_->edges.erase(it);  // unique_ptr destructor frees the Road
        removed = true;
    }
    return removed;
}

// tuning
void TrafficSimulator::set_target_vehicles(int n) {
    std::lock_guard<std::mutex> lock(mtx_);
    target_vehicles_ = n < 0 ? 0 : n;
}

void TrafficSimulator::set_spawn_rate(double per_second) {
    std::lock_guard<std::mutex> lock(mtx_);
    spawn_rate_ = per_second < 0.0 ? 0.0 : per_second;
}

bool TrafficSimulator::set_algorithm(const std::string& name) {
    if (name != "astar" && name != "dijkstra" && name != "bfs") return false;
    std::lock_guard<std::mutex> lock(mtx_);
    algorithm_ = name;
    return true;
}

// vehicle operations
bool TrafficSimulator::remove_vehicle(const std::string& id) {
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto it = vehicles_.begin(); it != vehicles_.end(); ++it) {
        if ((*it)->id != id) continue;
        destroy_vehicle_locked(it->get());
        vehicles_.erase(it);
        return true;
    }
    return false;
}

int TrafficSimulator::clear_vehicles() {
    std::lock_guard<std::mutex> lock(mtx_);
    const int n = static_cast<int>(vehicles_.size());
    for (auto& v : vehicles_) destroy_vehicle_locked(v.get());
    vehicles_.clear();
    return n;
}

bool TrafficSimulator::retarget_vehicle(const std::string& id,
                                        const std::string& destination) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!graph_) return false;
    auto dit = graph_->intersections.find(destination);
    if (dit == graph_->intersections.end()) return false;
    for (auto& v : vehicles_) {
        if (v->id != id) continue;
        v->recompute_route(graph_, dit->second.get());
        return !v->get_route().empty();
    }
    return false;
}

bool TrafficSimulator::set_vehicle_algorithm(const std::string& id,
                                             const std::string& algo) {
    if (algo != "astar" && algo != "dijkstra" && algo != "bfs") return false;
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto& v : vehicles_) {
        if (v->id != id) continue;
        v->set_algorithm(algo);
        if (v->destination) v->recompute_route(graph_, v->destination);
        return true;
    }
    return false;
}

namespace {
// Road ids of the part of the route still to be driven, for the selection card
// to highlight. Kept out of state_json on purpose: a route can be 60+ roads,
// and multiplying that by every vehicle on screen would dwarf the rest of the
// payload for data nobody is looking at except on the one selected vehicle.
std::string route_ids_json(const Vehicle& v) {
    std::ostringstream os;
    os << "[";
    bool first = true;
    const auto& route = v.get_route();
    for (std::size_t k = static_cast<std::size_t>(std::max(0, v.get_route_index()));
         k < route.size(); ++k) {
        if (!route[k]) continue;
        if (!first) os << ",";
        first = false;
        os << jstr(route[k]->id);
    }
    os << "]";
    return os.str();
}
}  // namespace

std::string TrafficSimulator::vehicle_detail_json(const std::string& id) {
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto& v : vehicles_) {
        if (v->id != id) continue;
        std::ostringstream os;
        os << "{\"id\":" << jstr(v->id)
           << ",\"type\":" << jstr(v->type())
           << ",\"algorithm\":" << jstr(v->algorithm())
           << ",\"speed\":" << v->current_speed
           << ",\"max_speed\":" << v->max_speed
           << ",\"road\":" << jstr(v->current_road ? v->current_road->id : "")
           << ",\"pos\":" << v->position_on_road
           << ",\"destination\":" << jstr(v->destination ? v->destination->name : "")
           << ",\"route_len\":" << v->get_route().size()
           << ",\"route_index\":" << v->get_route_index()
           << ",\"route\":" << route_ids_json(*v)
           << ",\"arrived\":" << (v->has_reached_destination() ? "true" : "false")
           << "}";
        return os.str();
    }
    return "{\"error\":\"no such vehicle\"}";
}

// live map editing
std::string TrafficSimulator::add_junction_at(double x, double y) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!graph_) return "";
    const std::string id = "j" + std::to_string(next_id_++);
    graph_->intersections[id] = std::make_unique<Intersection>(id, x, y);
    reindex_locked();
    return id;
}

std::string TrafficSimulator::split_road_near(double x, double y) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!graph_) return "";

    // Closest road, and where along it the click landed.
    Road* best = nullptr;
    double best_d = 1e18, best_t = 0.0;
    for (const auto& [rid, r] : graph_->edges) {
        if (!r->source || !r->target) continue;
        const double ax = r->source->x, ay = r->source->y;
        const double vx = r->target->x - ax, vy = r->target->y - ay;
        const double L2 = vx * vx + vy * vy;
        if (L2 < 1e-9) continue;
        double t = ((x - ax) * vx + (y - ay) * vy) / L2;
        if (t < 0.05) t = 0.05;          // keep both halves long enough to drive
        if (t > 0.95) t = 0.95;
        const double d = std::hypot(ax + vx * t - x, ay + vy * t - y);
        if (d < best_d) { best_d = d; best = r.get(); best_t = t; }
    }
    if (!best) return "";

    Intersection* a = best->source;
    Intersection* b = best->target;
    const int lanes = best->lanes;
    const double speed = best->speed_limit;
    const bool oneway = best->is_one_way;
    std::string base = best->id;
    if (base.size() > 2 && base.compare(base.size() - 2, 2, "_r") == 0) {
        base = base.substr(0, base.size() - 2);   // always split the forward id
    }

    const std::string mid_id = "j" + std::to_string(next_id_++);
    auto mid_owned = std::make_unique<Intersection>(mid_id,
                                                     a->x + (b->x - a->x) * best_t,
                                                     a->y + (b->y - a->y) * best_t);
    Intersection* mid = mid_owned.get();
    graph_->intersections[mid_id] = std::move(mid_owned);

    // Vehicles standing on the road about to be split must survive it. Record
    // where each one actually IS in the world, and which way it is pointing,
    // BEFORE the road is deleted -- afterwards there is nothing to measure
    // against.
    //
    // Position is captured as a world coordinate rather than as "fraction
    // along the road", because `base` and its "_r" twin run in opposite
    // directions and the split point is not necessarily in the middle. World
    // coordinates plus a heading are unambiguous; fractions are not, and
    // getting that wrong silently teleports traffic.
    struct Displaced { Vehicle* v; double x, y, dx, dy; };
    std::vector<Displaced> displaced;

    for (const std::string& rid : {base, base + "_r"}) {
        auto rit = graph_->edges.find(rid);
        if (rit == graph_->edges.end()) continue;
        Road* r = rit->second.get();
        if (!r->source || !r->target || r->length < 1e-9) continue;
        const double ux = (r->target->x - r->source->x) / r->length;
        const double uy = (r->target->y - r->source->y) / r->length;
        for (Vehicle* v : r->vehicles) {
            if (!v) continue;
            displaced.push_back({v,
                                 r->source->x + ux * v->position_on_road,
                                 r->source->y + uy * v->position_on_road,
                                 ux, uy});
            // Hide it from remove_road_locked(), which despawns anything still
            // claiming to be on the road it is deleting.
            v->current_road = nullptr;
        }
    }

    remove_road_locked(base);
    const std::string id_a = base + "_s" + std::to_string(next_id_++);
    const std::string id_b = base + "_s" + std::to_string(next_id_++);
    graph_->create_directional_road(id_a, a, mid, lanes, speed, oneway);
    graph_->create_directional_road(id_b, mid, b, lanes, speed, oneway);
    if (!oneway) {
        graph_->create_directional_road(id_a + "_r", mid, a, lanes, speed, oneway);
        graph_->create_directional_road(id_b + "_r", b, mid, lanes, speed, oneway);
    }
    
    // Put the traffic back, on whichever half now covers the spot it was
    // standing on and points the way it was going. A driver crossing this
    // point should not notice the edit at all: same place, same speed, same
    // lane.
    for (const Displaced& d : displaced) {
        Road* best_fit = nullptr;
        double best_pos = 0.0, best_dist = 1e18;
        for (const std::string& cand : {id_a, id_b, id_a + "_r", id_b + "_r"}) {
            auto it = graph_->edges.find(cand);
            if (it == graph_->edges.end()) continue;
            Road* r = it->second.get();
            if (!r->source || !r->target || r->length < 1e-9) continue;
            const double ux = (r->target->x - r->source->x) / r->length;
            const double uy = (r->target->y - r->source->y) / r->length;
            if (ux * d.dx + uy * d.dy < 0.0) continue;   // wrong way down it
            double t = (d.x - r->source->x) * ux + (d.y - r->source->y) * uy;
            const double clamped = std::clamp(t, 0.0, r->length);
            const double dist = std::hypot(r->source->x + ux * clamped - d.x,
                                           r->source->y + uy * clamped - d.y)
                              + std::abs(t - clamped);
            if (dist < best_dist) { best_dist = dist; best_fit = r; best_pos = clamped; }
        }
        if (!best_fit) {
            // Nothing to stand on -- rather than leave a vehicle pointing at a
            // road that is not there, let it go.
            destroy_vehicle_locked(d.v);
            vehicles_.erase(std::remove_if(vehicles_.begin(), vehicles_.end(),
                                           [&](const std::unique_ptr<Vehicle>& p) { return p.get() == d.v; }),
                            vehicles_.end());
            continue;
        }
        d.v->transfer_to_road(best_fit, best_pos);
    }

    reindex_locked();
    invalidate_routes_locked();
    return mid_id;
}

std::string TrafficSimulator::add_road_between(const std::string& from,
                                               const std::string& to,
                                               double speed_kmh, int lanes,
                                               bool oneway) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!graph_) return "";
    auto fit = graph_->intersections.find(from);
    auto tit = graph_->intersections.find(to);
    if (fit == graph_->intersections.end() || tit == graph_->intersections.end()) return "";
    if (from == to) return "";

    if (lanes < 1) lanes = 1;
    if (speed_kmh < 5.0) speed_kmh = 5.0;
    const std::string id = "r" + std::to_string(next_id_++);
    graph_->create_directional_road(id, fit->second.get(), tit->second.get(), lanes, speed_kmh, oneway);
    if (!oneway) {
        graph_->create_directional_road(id + "_r", tit->second.get(), fit->second.get(),
                                        lanes, speed_kmh, oneway);
    }
    reindex_locked();
    invalidate_routes_locked();
    return id;
}

bool TrafficSimulator::remove_road(const std::string& id) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!remove_road_locked(id)) return false;
    reindex_locked();
    invalidate_routes_locked();
    return true;
}

bool TrafficSimulator::remove_intersection(const std::string& id) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!graph_) return false;
    auto it = graph_->intersections.find(id);
    if (it == graph_->intersections.end()) return false;

    // Drop every road touching the node first, then the node itself.
    std::vector<std::string> doomed;
    for (const auto& [rid, r] : graph_->edges) {
        if ((r->source && r->source->name == id) ||
            (r->target && r->target->name == id)) {
            doomed.push_back(rid);
        }
    }
    for (const std::string& rid : doomed) remove_road_locked(rid);

    graph_->intersections.erase(it);  // unique_ptr destructor frees the Intersection
    reindex_locked();
    invalidate_routes_locked();
    return true;
}

bool TrafficSimulator::update_road(const std::string& id, double speed_kmh, bool closed) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!graph_) return false;
    bool touched = false;
    // Apply to both directions of a two-way street so they stay consistent.
    for (const std::string& rid : {id, id + "_r"}) {
        auto it = graph_->edges.find(rid);
        if (it == graph_->edges.end()) continue;
        if (speed_kmh > 0.0) it->second->speed_limit = speed_kmh;
        it->second->is_closed = closed;
        it->second->update();
        touched = true;
    }
    if (touched) invalidate_routes_locked();
    return touched;
}

bool TrafficSimulator::set_light(const std::string& node_id, bool on, double green) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!graph_) return false;
    auto it = graph_->intersections.find(node_id);
    if (it == graph_->intersections.end()) return false;

    if (!on) {
        graph_->designalize(node_id);
        return true;
    }
    if (green < 5.0) green = 5.0;
    // Go through signalize() so the approaches get phase groups and stop
    // lines. It refuses a junction with nothing to arbitrate, and the caller
    // is told rather than left with a light that holds every approach on red.
    return graph_->signalize(node_id, green);
}

std::string TrafficSimulator::add_building_at(double x, double y,
                                              const std::string& kind,
                                              const std::string& name,
                                              double size) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!graph_) return "";
    if (size < 4.0) size = 12.0;
    const double h = size * 0.5;
    std::vector<std::pair<double, double>> footprint{
        {x - h, y - h}, {x + h, y - h}, {x + h, y + h}, {x - h, y + h},
    };
    std::string key = name.empty() ? ("b" + std::to_string(next_id_++)) : name;
    while (graph_->buildings.count(key)) key += "_" + std::to_string(next_id_++);
    graph_->buildings[key] =
        std::make_unique<Building>(key, kind.empty() ? "yes" : kind, 1, std::move(footprint));
    return key;
}

bool TrafficSimulator::remove_building(const std::string& name) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!graph_) return false;
    auto it = graph_->buildings.find(name);
    if (it == graph_->buildings.end()) return false;
    graph_->buildings.erase(it);  // unique_ptr destructor frees the Building
    return true;
}

// map files
// Reject anything that could escape data/maps/.
static bool safe_map_name(const std::string& name) {
    if (name.empty()) return false;
    return name.find('/') == std::string::npos &&
           name.find('\\') == std::string::npos &&
           name.find("..") == std::string::npos;
}

std::string TrafficSimulator::maps_json() const {
    std::ostringstream os;
    os << "{\"maps\":[";
    bool first = true;
    std::error_code ec;
    std::filesystem::directory_iterator it("data/maps", ec), end;
    for (; !ec && it != end; it.increment(ec)) {
        if (!it->is_regular_file()) continue;
        if (it->path().extension() != ".geojson") continue;
        if (!first) os << ",";
        first = false;
        os << jstr(it->path().stem().string());
    }
    os << "]}";
    return os.str();
}

bool TrafficSimulator::save_map(const std::string& name) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!graph_ || !safe_map_name(name)) return false;
    try {
        MapLoader::write_geojson(*graph_, "data/maps/" + name + ".geojson");
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

TrafficSimulator::ImportResult TrafficSimulator::import_map(
        const std::string& name, const std::string& geojson) {
    if (!safe_map_name(name)) return {false, "bad map name"};
    if (geojson.empty()) return {false, "empty document"};

    // Parse into a graph of its own BEFORE touching anything. An import that
    // turns out to be malformed, or to contain no drivable roads, must leave
    // the city that is already running exactly as it was.
    std::unique_ptr<TrafficGraph> candidate(new TrafficGraph());
    try {
        MapLoader::load_from_geojson_string(*candidate, geojson, "imported map");
    } catch (const std::exception& e) {
        return {false, e.what()};
    }
    const std::string warning = MapLoader::validate(*candidate);
    candidate.reset();   // load() below re-parses; this was only the gate

    // Known good, so it is worth keeping. Writing it out means the area can be
    // reloaded later from the map list without going back to OpenStreetMap.
    const std::string path = "data/maps/" + name + ".geojson";
    try {
        std::ofstream out(path, std::ios::binary);
        if (!out) return {false, "could not write " + path};
        out << geojson;
        out.close();
    } catch (const std::exception& e) {
        return {false, e.what()};
    }

    try {
        load(path);
    } catch (const std::exception& e) {
        return {false, e.what()};
    }
    // Loaded. `warning` is empty unless validate() had something to say, in
    // which case the map is usable but worth complaining about.
    return {true, warning};
}

bool TrafficSimulator::load_map(const std::string& name) {
    if (!safe_map_name(name)) return false;
    try {
        load("data/maps/" + name + ".geojson");  // takes the lock itself
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

// ==========================================================================
// Per-tick traffic solver (M3/M4 lane changes, M6 yielding, M7 gridlock)
//
// Vehicles cannot see each other: a Vehicle only knows its own road and
// position. Everything that needs a global view lives here and is handed to
// Vehicle::advance() as an explicit (leader, gap) pair once per tick.
// ==========================================================================

namespace {
    // ---- MOBIL (lane-change decision model) ---------------------------------
    constexpr double kMobilSafeDecel = -2.0;       // new follower may brake this hard
    constexpr double kMobilThreshold = 0.35;       // min gain to bother (m/s^2)
    constexpr double kMobilReturnThreshold = 1.0;  // going back needs a bigger win
    constexpr double kMobilObstruction = 0.4;      // only change if actually held up

    // ---- junction yielding (M6) ---------------------------------------------
    constexpr double kJunctionZone = 2.0;    // m: conflict radius around the node
    // The distance at which yielding starts is NOT a constant -- it depends on
    // how big the map's blocks are. See TrafficSimulator::junction_approach_
    // and set_junction_approach(); the ceiling below is what the reference uses
    // on its 240 m downtown blocks and stays the upper bound.
    constexpr double kZoneApproachMax = 26.0;  // m
    constexpr double kZoneHorizon = 4.0;     // s: conflicts farther out are ignored
    constexpr double kZoneTieMargin = 0.3;   // s: near-simultaneous arrival window
    constexpr double kZoneBonusCap = 3.0;    // s of head start impatience may claim
    constexpr double kMovingSpeed = 0.5;     // m/s above which a vehicle is moving
    constexpr double kZoneMinSpeed = 1.5;    // m/s floor when estimating arrival
    constexpr double kIgnoreBlockerAfter = 15.0;  // s stuck -> stop giving way

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
}

// Nearest vehicle ahead of `v` in the same lane, looking past the end of the
// current road when nothing blocks us on it.
// Returns {leader, gap}; leader is null when the road ahead is clear.
std::pair<Vehicle*, double> TrafficSimulator::lead_in_lane(Vehicle* v) const {
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
    const std::vector<int> allowed = v->allowed_lanes(next->lanes);
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
double TrafficSimulator::accel_in_lane(Vehicle* v, int lane) const {
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
Vehicle* TrafficSimulator::follower_in_lane(Vehicle* v, int lane) const {
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
void TrafficSimulator::solve_lane_changes(double sim_time) {
    for (auto& vp : vehicles_) {
        Vehicle* v = vp.get();
        Road* road = v->current_road;
        if (!road || road->lanes < 2) continue;
        if (!v->lane_change_ready(sim_time)) continue;

        const std::vector<int> allowed = v->allowed_lanes(road->lanes);
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

            const double a_new = accel_in_lane(v, lane);
            double gain = a_new - a_cur;

            // Extra incentive for queued/stationary vehicles to escape blocked lanes if target lane is clear ahead
            if (v->current_speed < 1.0) {
                double cur_gap = 1e9, target_gap = 1e9;
                for (Vehicle* o : road->vehicles) {
                    if (o == v || o->position_on_road <= v->position_on_road) continue;
                    const double g = gap_between(v, o);
                    if (o->lane == v->lane && g < cur_gap) cur_gap = g;
                    if (o->lane == lane && g < target_gap) target_gap = g;
                }
                if (target_gap > cur_gap + 3.0) gain += 1.5;
            }

            // Politeness: subtract the disadvantage we would inflict on the
            // vehicle that ends up behind us. politeness 0 ignores it entirely.
            Vehicle* follower = follower_in_lane(v, lane);
            if (follower) {
                const double f_before = accel_in_lane(follower, lane);
                const double f_after = Vehicle::idm_accel(
                    follower->current_speed, follower->desired_speed_on(*road),
                    v->current_speed, std::max(0.0, gap_between(follower, v)));
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
void TrafficSimulator::solve_junctions(
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

        const bool signalized = node->is_signalized && node->light_system;

        const double to_end = road->length - v->position_on_road;
        if (to_end >= junction_approach_ || to_end <= kJunctionZone) continue;

        const uint32_t my_base_idx = road->base_index;
        const Road* my_next =
            (v->route_index + 1 < static_cast<int>(v->route.size()))
                ? v->route[v->route_index + 1]
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

void TrafficSimulator::break_cycles(const std::vector<int>& waiting_on) {
    std::vector<double> stuck(vehicles_.size());
    for (std::size_t i = 0; i < vehicles_.size(); ++i) {
        stuck[i] = vehicles_[i]->stuck_time;
    }

    for (std::size_t leader : find_gridlock_cycle_leaders(waiting_on, stuck)) {
        Vehicle* v = vehicles_[leader].get();
        if (v->is_forced(sim_time_)) continue;   // already being pushed through
        v->grant_forced_clearance(sim_time_ + kForcedGrant);
        forced_throughs_++;
        controller_.log_line("Gridlock broken: " + v->id + " forced through");
    }
}

// M7 stage 2: the vehicle is not in a cycle we can name, and it has been
// standing still for the best part of a minute. Lift it forward along its own
// route. This is the escape hatch of last resort -- if it runs often, the
// traffic model is wrong and this is papering over it.

void TrafficSimulator::solve_gridlock(double sim_time) {
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
