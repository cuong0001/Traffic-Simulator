#include "simulation/TrafficSimulator.hpp"

#include <algorithm>
#include <chrono>
#include <thread>
#include <cmath>
#include <cstring>
#include <limits>
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

    map_editor_.reindex();

    stats_ = StatisticsManager();
    sim_time_ = 0.0;
    sim_clock_base_epoch_ = std::chrono::duration<double>(
                                 std::chrono::system_clock::now().time_since_epoch())
                                 .count();
    tick_count_ = 0;
    next_id_ = 1;
    spawn_accum_ = 0.0;
    sample_timer_ = 0.0;
    arbiter_.reset_counters();
    // Re-derived on every load, deliberately: the right give-way distance is a
    // property of the city you are driving in, not of the session. Carrying a
    // hand-tuned 12 m onto a map with 200 m blocks would have cars deciding
    // almost inside the junction, and nothing on screen would explain why.
    arbiter_.set_junction_approach(default_junction_approach(*graph_));
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
    arbiter_.set_junction_approach(metres);
}

double TrafficSimulator::junction_approach() const {
    return arbiter_.junction_approach();
}

// ==========================================================================
// The tick loop, in five phases. See the header for why the order matters.
// ==========================================================================
void TrafficSimulator::tick(double dt) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (paused_ || !graph_) return;
    dt *= speed_;

    tick_events_and_lanes(dt);
    tick_leads_and_physics(dt);
    tick_cleanup_and_spawn(dt);
    tick_statistics(dt);
    tick_history_capture(dt);

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
        if (!v->can_recompute() || !v->get_destination()) continue;
        if (!v->consume_reroute_flag()) continue;
        const auto t0 = std::chrono::steady_clock::now();
        v->recompute_route(graph_, v->get_destination());
        const double micros = std::chrono::duration<double, std::micro>(
                                  std::chrono::steady_clock::now() - t0).count();
        stats_.recordRouteComputation(v->algorithm(), micros, v->last_nodes_explored(), !v->get_route().empty());
        reroutes_this_tick++;
    }

    arbiter_.solve_lane_changes(sim_time_);
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
    for (auto& v : vehicles_) leaders.push_back(arbiter_.lead_in_lane(v.get()));

    std::vector<int> waiting_on(vehicles_.size(), -1);

    for (std::size_t i = 0; i < vehicles_.size(); ++i) {
        // Queued behind a car that is not moving: a wait-for edge.
        Vehicle* lead = leaders[i].first;
        if (lead && leaders[i].second < kWaitEdgeGap &&
            lead->current_speed < kStoppedSpeed) {
            waiting_on[i] = static_cast<int>(lead->sim_index);
        }
    }

    arbiter_.solve_junctions(dt, sim_time_, leaders, waiting_on);
    arbiter_.solve_gridlock(sim_time_);

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

// Phase 5: persist one frame for the history scrubber, once per REAL second
// -- deliberately NOT gated on `dt` (which is already scaled by speed_ by the
// time it reaches here): capturing on simulated time would make total data
// volume for the same real 24h window vary by up to 20x depending on what
// speed the map happened to run at, defeating the point of a size-bounded
// window. See HistoryStore.hpp's own comment for the full reasoning.
//
// This walks the same live objects state_json() does, into the shape
// HistoryStore expects (see HistoryStore.hpp). The two walks are NOT shared
// code -- state_json() stays untouched to avoid risking its many existing
// callers/tests for this -- so if a field's meaning ever changes on one side
// (e.g. a new light attribute), check the other.
void TrafficSimulator::tick_history_capture(double /*dt*/) {
    const double now = std::chrono::duration<double>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count();
    if (now - last_history_capture_real_ < 1.0) return;
    last_history_capture_real_ = now;

    HistoryFrame f;
    f.real_time = now;
    f.sim_time = sim_time_;
    f.calendar_time = sim_clock_base_epoch_ + sim_time_;

    f.vehicles.reserve(vehicles_.size());
    for (const auto& v : vehicles_) {
        if (!v->current_road) continue;
        HistoryVehicleRecord r;
        r.id = v->id;
        r.road = v->current_road->id;
        r.pos = static_cast<float>(v->position_on_road);
        r.speed = static_cast<float>(v->current_speed);
        r.lane = v->lane;
        r.lateral_offset = static_cast<float>(v->lateral_offset);
        r.recalcs = v->recalculations();
        r.state = static_cast<int>(v->state());
        r.type = v->type();
        f.vehicles.push_back(std::move(r));
    }

    if (graph_) {
        for (const auto& [id, road] : graph_->edges) {
            if (road->signal_group < 0 || !road->target) continue;
            if (!road->target->is_signalized()) continue;
            HistoryLightRecord l;
            l.id = id;
            l.junction = road->target->name;
            l.group = road->signal_group;
            l.stop_from_end = static_cast<float>(road->signal_stop_from_end);
            l.state = road->target->signal_state_for(road->signal_group, sim_time_);
            l.has_head = road->has_signal_head;
            if (l.has_head) {
                l.hx = static_cast<float>(road->signal_head_x);
                l.hy = static_cast<float>(road->signal_head_y);
                l.hux = static_cast<float>(road->signal_head_ux);
                l.huy = static_cast<float>(road->signal_head_uy);
            }
            f.lights.push_back(std::move(l));
        }
        for (const auto& [id, road] : graph_->edges) {
            if (road->get_congestion_level() <= 0.01 && !road->is_closed()) continue;
            f.roads.push_back({id, static_cast<float>(road->get_congestion_level()),
                                road->is_closed()});
        }
    }

    for (const auto& e : controller_.events()) {
        f.events.push_back({e->id(), e->type(), e->target(), e->description(), e->remaining()});
    }

    double total_cong = 0.0;
    if (graph_) {
        for (const auto& [id, road] : graph_->edges) {
            if (road) total_cong += road->get_congestion_level();
        }
    }
    f.stats.avg_congestion = (graph_ && !graph_->edges.empty())
                                  ? total_cong / static_cast<double>(graph_->edges.size())
                                  : 0.0;
    f.stats.active_vehicles = static_cast<int>(vehicles_.size());
    f.stats.avg_travel_time = stats_.averageTravelTime();
    f.stats.completed_trips = stats_.completedTrips();
    f.stats.spawned = next_id_ - 1;

    history_.capture(f);
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
    v->set_cost_mode(cost_mode_);
    const auto t0 = std::chrono::steady_clock::now();
    v->request_route(graph_, fit->second.get(), tit->second.get());
    const double micros = std::chrono::duration<double, std::micro>(
                              std::chrono::steady_clock::now() - t0).count();
    stats_.recordRouteComputation(algorithm_, micros, v->last_nodes_explored(), !v->get_route().empty());
    if (v->get_route().empty()) {  // unreachable
        return "";
    }
    // request_route() already put v on its first road. Spread spawns across
    // every allowed lane whose entrance is clear, preferring whichever is
    // least occupied so multi-lane roads actually get used as multi-lane.
    //
    // The previous rule kept the candidate with the largest "clearest gap
    // ahead" (min_pos), initialised to 1e9 and compared with a strict `>`. On
    // an empty or lightly used road every lane ties at exactly 1e9, so the
    // first candidate always won -- every vehicle in the city spawned in
    // lane 0, and because MOBIL only moves a vehicle that is actually held
    // up, it stayed there. A 3-lane road ran as a 1-lane road.
    if (Road* first = v->current_road) {
        const std::vector<int> ok = v->allowed_lanes(first->get_lanes());
        const double needed = v->length_meters() + kSpawnClearance;

        std::vector<int> best_lanes;
        std::size_t fewest = std::numeric_limits<std::size_t>::max();

        for (int l : ok) {
            bool slot_ok = true;
            std::size_t occupants = 0;
            for (Vehicle* o : first->vehicles) {
                if (o == v || o->lane != l) continue;
                ++occupants;
                if (o->position_on_road < needed) { slot_ok = false; break; }
            }
            if (!slot_ok) continue;
            if (occupants < fewest) { fewest = occupants; best_lanes.clear(); }
            if (occupants == fewest) best_lanes.push_back(l);
        }

        if (best_lanes.empty()) {  // every allowed lane blocked at the entrance
            first->remove_vehicle(v);
            return "";
        }

        std::uniform_int_distribution<std::size_t> pick(0, best_lanes.size() - 1);
        v->lane = best_lanes[pick(rng_)];
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
    speed_ = std::max(0.1, std::min(10.0, factor));
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
    const auto& node_ids = map_editor_.node_ids();
    if (node_ids.empty()) return "";
    std::uniform_int_distribution<std::size_t> dist(0, node_ids.size() - 1);
    return node_ids[dist(rng_)];
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
    // Wipes any playback history a PRIOR run of the simulator left on disk
    // -- once per process, here, since this is already the guarded "run
    // exactly once at boot" entry point and adding a second one just for
    // this would be one more thing main() has to remember to call in the
    // right order.
    history_.reset();
    // Detached, like the tick thread in main(): the simulator is a singleton
    // that lives as long as the process, so there is no shutdown to join at.
    std::thread(&TrafficSimulator::publish_loop, this).detach();
}

// ---- history playback -----------------------------------------------------
std::string TrafficSimulator::history_range_json() {
    std::lock_guard<std::mutex> lock(mtx_);
    const HistoryStore::Range r = history_.range();
    std::ostringstream os;
    os << "{\"empty\":" << (r.empty ? "true" : "false")
       << ",\"oldestAgo\":" << r.oldest_ago
       << ",\"newestAgo\":" << r.newest_ago
       << ",\"windowSeconds\":" << r.window_seconds << "}";
    return os.str();
}

std::string TrafficSimulator::history_state_at_json(double seconds_ago) {
    std::lock_guard<std::mutex> lock(mtx_);
    auto frame = history_.at(seconds_ago);
    if (!frame) {
        return "{\"empty\":true,\"vehicles\":[],\"lights\":[],\"roads\":[],\"events\":[],"
               "\"active\":0,\"spawned\":0,"
               "\"stats\":{\"avgCongestion\":0,\"activeVehicles\":0,\"avgTravelTime\":0,"
               "\"completedTrips\":0}}";
    }
    return HistoryStore::to_state_json(*frame);
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

// tick_history_capture() (below tick_statistics()) walks the same vehicles_/
// graph_/controller_ into a HistoryFrame, on its own 1s-real-time cadence.
// The two extraction walks are separate code, not shared -- this one has too
// many existing callers/tests to risk refactoring for that -- so a field
// meaning changing on one side (e.g. a new light attribute) needs checking
// against the other by hand.
std::string TrafficSimulator::state_json() {
    std::lock_guard<std::mutex> lock(mtx_);
    std::ostringstream os;
    os << "{";
    os << "\"time\":" << sim_time_ << ",";
    // Cast to an integer before insertion: ostringstream defaults to 6
    // significant digits for a double, which for a ~1.7-billion-second epoch
    // value throws away everything below ~10000s resolution -- the clock
    // would read up to ~3h wrong. Whole seconds are all a clock display needs.
    os << "\"calendarTime\":" << static_cast<long long>(sim_clock_base_epoch_ + sim_time_) << ",";
    os << "\"paused\":" << (paused_ ? "true" : "false") << ",";
    os << "\"speed\":" << speed_ << ",";
    // The control panel shows these. Without them in the state the inputs can
    // only display whatever was hard-coded in index.html, which drifts from
    // what the simulator is actually doing the moment either side changes.
    os << "\"targetVehicles\":" << target_vehicles_ << ",";
    os << "\"spawnRate\":" << spawn_rate_ << ",";
    os << "\"junctionApproach\":" << arbiter_.junction_approach() << ",";
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
            if (!road->target->is_signalized()) continue;
            if (!first) os << ",";
            first = false;
            os << "{\"id\":" << jstr(id)
               << ",\"n\":" << jstr(road->target->name)
               << ",\"g\":" << road->signal_group
               << ",\"d\":" << road->signal_stop_from_end;
            // Where the source map actually put this head, when it said so.
            // Approaches without one still report their state (they still stop
            // traffic) but the renderer draws no head for them.
            if (road->has_signal_head) {
                os << ",\"hx\":" << road->signal_head_x
                   << ",\"hy\":" << road->signal_head_y
                   << ",\"hux\":" << road->signal_head_ux
                   << ",\"huy\":" << road->signal_head_uy;
            }
            os << ",\"state\":"
               << road->target->signal_state_for(road->signal_group, sim_time_) << "}";
        }
    }
    os << "],";

    // congested / closed roads only (keep the payload small)
    os << "\"roads\":[";
    first = true;
    if (graph_) {
        for (const auto& [id, road] : graph_->edges) {
            if (road->get_congestion_level() <= 0.01 && !road->is_closed()) continue;
            if (!first) os << ",";
            first = false;
            os << "{\"id\":" << jstr(id)
               << ",\"congestion\":" << road->get_congestion_level()
               << ",\"closed\":" << (road->is_closed() ? "true" : "false") << "}";
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

    os << "\"solver\":{\"lane_changes\":" << arbiter_.lane_changes()
       << ",\"yields\":" << arbiter_.yields()
       << ",\"box_waits\":" << arbiter_.box_waits()
       << ",\"forced\":" << arbiter_.forced_throughs()
       << ",\"rescues\":" << arbiter_.rescues()
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

void TrafficSimulator::set_cost_mode(CostMode mode) {
    std::lock_guard<std::mutex> lock(mtx_);
    cost_mode_ = mode;
    for (auto& v : vehicles_) v->set_cost_mode(mode);
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
        if (v->get_destination()) v->recompute_route(graph_, v->get_destination());
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
           << ",\"max_speed\":" << v->get_max_speed()
           << ",\"road\":" << jstr(v->current_road ? v->current_road->id : "")
           << ",\"pos\":" << v->position_on_road
           << ",\"destination\":" << jstr(v->get_destination() ? v->get_destination()->name : "")
           << ",\"route_len\":" << v->get_route().size()
           << ",\"route_index\":" << v->get_route_index()
           << ",\"route\":" << route_ids_json(*v)
           << ",\"arrived\":" << (v->has_reached_destination() ? "true" : "false")
           << "}";
        return os.str();
    }
    return "{\"error\":\"no such vehicle\"}";
}

// live map editing -- one-line forwarders to MapEditor, which does the actual
// work. Locking stays here: MapEditor assumes the caller already holds mtx_,
// the same convention TrafficSimulator's own "_locked" helpers use.
std::string TrafficSimulator::add_junction_at(double x, double y) {
    std::lock_guard<std::mutex> lock(mtx_);
    return map_editor_.add_junction_at(x, y);
}

std::string TrafficSimulator::split_road_near(double x, double y) {
    std::lock_guard<std::mutex> lock(mtx_);
    return map_editor_.split_road_near(x, y);
}

std::string TrafficSimulator::add_road_between(const std::string& from,
                                               const std::string& to,
                                               double speed_kmh, int lanes,
                                               bool oneway) {
    std::lock_guard<std::mutex> lock(mtx_);
    return map_editor_.add_road_between(from, to, speed_kmh, lanes, oneway);
}

bool TrafficSimulator::remove_road(const std::string& id) {
    std::lock_guard<std::mutex> lock(mtx_);
    return map_editor_.remove_road(id);
}

bool TrafficSimulator::remove_intersection(const std::string& id) {
    std::lock_guard<std::mutex> lock(mtx_);
    return map_editor_.remove_intersection(id);
}

bool TrafficSimulator::update_road(const std::string& id, double speed_kmh, bool closed) {
    std::lock_guard<std::mutex> lock(mtx_);
    return map_editor_.update_road(id, speed_kmh, closed);
}

bool TrafficSimulator::set_light(const std::string& node_id, bool on, double green) {
    std::lock_guard<std::mutex> lock(mtx_);
    return map_editor_.set_light(node_id, on, green);
}

std::string TrafficSimulator::add_building_at(double x, double y,
                                              const std::string& kind,
                                              const std::string& name,
                                              double size) {
    std::lock_guard<std::mutex> lock(mtx_);
    return map_editor_.add_building_at(x, y, kind, name, size);
}

bool TrafficSimulator::remove_building(const std::string& name) {
    std::lock_guard<std::mutex> lock(mtx_);
    return map_editor_.remove_building(name);
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


// M7: break deadlocks by finding cycles in the wait-for graph built by
// arbiter_.solve_junctions(). See JunctionArbiter.hpp/.cpp for the actual
// decision.
void TrafficSimulator::break_cycles(const std::vector<int>& waiting_on) {
    arbiter_.break_cycles(waiting_on, sim_time_);
}
