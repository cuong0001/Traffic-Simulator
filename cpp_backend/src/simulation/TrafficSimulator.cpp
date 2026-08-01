#include "simulation/TrafficSimulator.hpp"

#include <algorithm>
#include <chrono>
#include <thread>
#include <cmath>
#include <unordered_map>
#include <filesystem>
#include <sstream>
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
    constexpr double kWaitEdgeGap = 12.0;      // m: closer than this and we are queued
    constexpr double kStoppedSpeed = 0.5;      // m/s below which a vehicle is stopped
}

// Singleton 
TrafficSimulator& TrafficSimulator::instance() {
    static TrafficSimulator sim; 
    return sim;
}

TrafficSimulator::~TrafficSimulator() { clear(); }

void TrafficSimulator::clear() {
    // Detach before deleting. The controller keeps raw IObserver* pointers, so
    // a vehicle freed while still subscribed leaves a dangling entry that the
    // next notifyAll() would call into.
    for (Vehicle* v : vehicles_) {
        controller_.detach(v);
        delete v;
    }
    vehicles_.clear();
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
    lane_changes_ = yields_ = forced_throughs_ = rescues_ = 0;
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

    // Replan anyone an event flagged. Done here rather than inside onNotify()
    // because that fires mid-broadcast, while the route is being iterated.
    for (Vehicle* v : vehicles_) {
        if (!v->consume_reroute_flag() || !v->destination) continue;
        const auto t0 = std::chrono::steady_clock::now();
        v->recompute_route(graph_, v->destination);
        const double micros = std::chrono::duration<double, std::micro>(
                                  std::chrono::steady_clock::now() - t0).count();
        stats_.recordRouteComputation(v->algorithm(), micros, 0, !v->get_route().empty());
    }

    solve_lane_changes(sim_time_);
}

// Phase 2: work out who is in front of whom, decide right of way, then move
// everyone. Leaders are snapshotted first -- if we resolved them lazily, the
// first vehicle to move would change what the second one sees this tick.
void TrafficSimulator::tick_leads_and_physics(double dt) {
    solve_junctions(dt, sim_time_);
    solve_gridlock(sim_time_);

    std::vector<std::pair<Vehicle*, double>> leaders;
    leaders.reserve(vehicles_.size());
    for (Vehicle* v : vehicles_) leaders.push_back(lead_in_lane(v));

    // The wait-for graph, built as we apply the junction decisions: entry i is
    // the ONE vehicle that vehicle i is stuck behind, or -1. Because each
    // vehicle waits on at most one other, the graph is functional and a
    // deadlock is exactly a cycle in it.
    std::unordered_map<const Vehicle*, int> index_of;
    index_of.reserve(vehicles_.size() * 2);
    for (std::size_t i = 0; i < vehicles_.size(); ++i) index_of[vehicles_[i]] = static_cast<int>(i);

    std::vector<int> waiting_on(vehicles_.size(), -1);

    for (std::size_t i = 0; i < vehicles_.size(); ++i) {
        // Queued behind a car that is not moving: a wait-for edge.
        Vehicle* lead = leaders[i].first;
        if (lead && leaders[i].second < kWaitEdgeGap &&
            lead->current_speed < kStoppedSpeed) {
            auto at = index_of.find(lead);
            if (at != index_of.end()) waiting_on[i] = at->second;
        }

        auto it = zone_stop_gap_.find(vehicles_[i]);
        if (it != zone_stop_gap_.end() && it->second < leaders[i].second) {
            leaders[i] = {nullptr, it->second};
            // Giving way to a parked occupant outranks the queue edge: it is
            // the junction, not the car in front, that we are actually stuck on.
            auto bl = zone_blocker_.find(vehicles_[i]);
            if (bl != zone_blocker_.end()) {
                auto at = index_of.find(bl->second);
                if (at != index_of.end()) waiting_on[i] = at->second;
            }
        }
    }

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
        Vehicle* v = *it;
        if (v->has_reached_destination()) {
            double length = 0.0;
            for (Road* r : v->get_route()) length += r->length;
            stats_.recordTripCompleted(v->travel_time(), length, v->type());
            controller_.detach(v);
            delete v;
            it = vehicles_.erase(it);
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

    const std::string id = type + std::to_string(next_id_++);
    // The factory hands back a unique_ptr; we take ownership into vehicles_,
    // which is freed by destroy_vehicle_locked() / clear().
    Vehicle* v = VehicleFactory::create(type, id).release();
    v->set_algorithm(algorithm_);
    const auto t0 = std::chrono::steady_clock::now();
    v->request_route(graph_, fit->second, tit->second);
    const double micros = std::chrono::duration<double, std::micro>(
                              std::chrono::steady_clock::now() - t0).count();
    stats_.recordRouteComputation(algorithm_, micros, 0, !v->get_route().empty());
    if (v->get_route().empty()) {  // unreachable
        delete v;
        return "";
    }
    // request_route() already put v on its first road, so skip v itself here.
    // Refuse to stack the new vehicle on one that is still queued at the kerb.
    if (Road* first = v->current_road) {
        const double needed = v->length_meters() + kSpawnClearance;
        for (Vehicle* o : first->vehicles) {
            if (o == v || o->lane != v->lane) continue;
            if (o->position_on_road < needed) {
                first->remove_vehicle(v);
                delete v;
                return "";
            }
        }
    }
    controller_.attach(v);  // Observer: v now hears about events
    vehicles_.push_back(v);
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
        // 20 Hz: faster than the UI polls, so a request never waits for the
        // next build, and slow enough that publishing costs a few percent of
        // one core rather than competing with the simulation.
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
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

    // vehicles
    os << "\"vehicles\":[";
    bool first = true;
    for (Vehicle* v : vehicles_) {
        if (!v->current_road) continue;
        if (!first) os << ",";
        first = false;
        os << "{\"id\":" << jstr(v->id)
           << ",\"road\":" << jstr(v->current_road->id)
           << ",\"pos\":" << v->position_on_road
           << ",\"speed\":" << v->current_speed
           << ",\"lane\":" << v->lane
           << ",\"lat\":" << v->lateral_offset
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
}

void TrafficSimulator::destroy_vehicle_locked(Vehicle* v) {
    if (!v) return;
    if (v->current_road) v->current_road->remove_vehicle(v);
    controller_.detach(v);
    delete v;
}

void TrafficSimulator::invalidate_routes_locked() {
    // The graph itself changed under their wheels. This is not one of the
    // TrafficNotification kinds (those describe events on an existing network),
    // so ask each vehicle directly instead of inventing a Kind for it.
    for (Vehicle* v : vehicles_) v->request_reroute();
}

// Drop a road and its reverse twin, detaching any vehicles on them.
// Caller holds mtx_ and is responsible for reindex/invalidate.
bool TrafficSimulator::remove_road_locked(const std::string& id) {
    if (!graph_) return false;
    bool removed = false;
    for (const std::string& rid : {id, id + "_r"}) {
        auto it = graph_->edges.find(rid);
        if (it == graph_->edges.end()) continue;
        Road* road = it->second;

        // Vehicles standing on a deleted road have nowhere to be: despawn them.
        for (auto vit = vehicles_.begin(); vit != vehicles_.end();) {
            if ((*vit)->current_road == road) {
                (*vit)->current_road = nullptr;   // already dying with the road
                destroy_vehicle_locked(*vit);
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
        graph_->edges.erase(it);
        delete road;
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
        destroy_vehicle_locked(*it);
        vehicles_.erase(it);
        return true;
    }
    return false;
}

int TrafficSimulator::clear_vehicles() {
    std::lock_guard<std::mutex> lock(mtx_);
    const int n = static_cast<int>(vehicles_.size());
    for (Vehicle* v : vehicles_) destroy_vehicle_locked(v);
    vehicles_.clear();
    return n;
}

bool TrafficSimulator::retarget_vehicle(const std::string& id,
                                        const std::string& destination) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!graph_) return false;
    auto dit = graph_->intersections.find(destination);
    if (dit == graph_->intersections.end()) return false;
    for (Vehicle* v : vehicles_) {
        if (v->id != id) continue;
        v->recompute_route(graph_, dit->second);
        return !v->get_route().empty();
    }
    return false;
}

bool TrafficSimulator::set_vehicle_algorithm(const std::string& id,
                                             const std::string& algo) {
    if (algo != "astar" && algo != "dijkstra" && algo != "bfs") return false;
    std::lock_guard<std::mutex> lock(mtx_);
    for (Vehicle* v : vehicles_) {
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
    for (Vehicle* v : vehicles_) {
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
    graph_->intersections[id] = new Intersection(id, x, y);
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
        if (d < best_d) { best_d = d; best = r; best_t = t; }
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
    Intersection* mid = new Intersection(mid_id,
                                         a->x + (b->x - a->x) * best_t,
                                         a->y + (b->y - a->y) * best_t);
    graph_->intersections[mid_id] = mid;

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
        Road* r = rit->second;
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
            Road* r = it->second;
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
            vehicles_.erase(std::remove(vehicles_.begin(), vehicles_.end(), d.v),
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
    graph_->create_directional_road(id, fit->second, tit->second, lanes, speed_kmh, oneway);
    if (!oneway) {
        graph_->create_directional_road(id + "_r", tit->second, fit->second,
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

    delete it->second;
    graph_->intersections.erase(it);
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
    Intersection* n = it->second;

    (void)n;
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
        new Building(key, kind.empty() ? "yes" : kind, 1, std::move(footprint));
    return key;
}

bool TrafficSimulator::remove_building(const std::string& name) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!graph_) return false;
    auto it = graph_->buildings.find(name);
    if (it == graph_->buildings.end()) return false;
    delete it->second;
    graph_->buildings.erase(it);
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
    constexpr double kJunctionZone = 6.0;    // m: conflict radius around the node
    constexpr double kZoneApproach = 26.0;   // m at which yielding starts
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
        return front->position_on_road - back->position_on_road - front->length_meters();
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
    for (Vehicle* v : vehicles_) {
        Road* road = v->current_road;
        if (!road || road->lanes < 2) continue;
        if (!v->lane_change_ready(sim_time) || v->current_speed < 1.0) continue;

        const std::vector<int> allowed = v->allowed_lanes(road->lanes);
        if (allowed.size() < 2) continue;

        const double a_cur = accel_in_lane(v, v->lane);
        const double a_free = Vehicle::idm_accel(v->current_speed,
                                                 v->desired_speed_on(*road), 0.0, 1e9);
        // Not actually held up? Then there is nothing to gain by weaving.
        if (a_cur >= a_free - kMobilObstruction) continue;

        int best_lane = v->lane;
        double best_gain = kMobilThreshold;

        for (int lane : allowed) {
            if (lane == v->lane || std::abs(lane - v->lane) != 1) continue;

            const double a_new = accel_in_lane(v, lane);
            double gain = a_new - a_cur;

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
void TrafficSimulator::solve_junctions(double dt, double sim_time) {
    zone_stop_gap_.clear();
    zone_blocker_.clear();

    // "w42_3" and "w42_3_r" are the two directions of one physical street.
    auto base_road = [](const std::string& id) {
        return id.size() > 2 && id.compare(id.size() - 2, 2, "_r") == 0
                   ? id.substr(0, id.size() - 2)
                   : id;
    };
    // Waypoints are shape points the loader inserts so roads follow street
    // curves, and roundabouts have their own priority rule. Neither is a place
    // where two streams cross, so neither arbitrates anything.
    auto is_crossing = [](const Intersection* n) {
        return n && !n->is_roundabout && n->zone != "waypoint";
    };

    // ---- pre-pass: who is IN each box, and who is approaching it ----------
    // Occupant = actively crossing (moving, within kJunctionZone before the
    // node) or still clearing it (within kJunctionZone past it). A vehicle
    // merely STOPPED at the stop line is not an occupant -- otherwise a queue
    // tail would block the cross street forever.
    std::unordered_map<const Intersection*,
                       std::vector<std::pair<std::size_t, Road*>>> occupants;
    std::unordered_map<const Intersection*,
                       std::vector<std::pair<double, std::size_t>>> approaching;
    for (std::size_t i = 0; i < vehicles_.size(); ++i) {
        const Vehicle* w = vehicles_[i];
        Road* r = w->current_road;
        if (!r) continue;
        const double to_end = r->length - w->position_on_road;
        const bool moving = w->current_speed > kMovingSpeed;

        if (is_crossing(r->target)) {
            if (to_end < kJunctionZone &&
                (moving || to_end < kJunctionZone - 2.0)) {
                occupants[r->target].push_back({i, r});
            } else if (to_end < kZoneApproach) {
                approaching[r->target].push_back({to_end, i});
            }
        }
        if (is_crossing(r->source) && w->position_on_road < kJunctionZone &&
            (moving || w->position_on_road < kJunctionZone - 2.0)) {
            occupants[r->source].push_back({i, r});
        }
    }

    // ---- decide, one vehicle at a time ------------------------------------
    for (std::size_t i = 0; i < vehicles_.size(); ++i) {
        Vehicle* v = vehicles_[i];
        Road* road = v->current_road;
        if (!road) continue;
        Intersection* node = road->target;
        if (!is_crossing(node)) continue;
        // A signalized junction is already arbitrated by its lights.
        if (node->is_signalized && node->light_system) continue;
        // Sirens do not queue, and neither does anyone the gridlock breaker
        // has already waved through, or who has been stuck so long that
        // giving way is the thing keeping them stuck.
        if (v->ignores_lights() || v->is_forced(sim_time)) continue;
        if (v->stuck_time > kIgnoreBlockerAfter) continue;

        // Only decide on the APPROACH. Past the stop line the vehicle is
        // committed: stopping there would leave it parked across the box.
        const double to_end = road->length - v->position_on_road;
        if (to_end >= kZoneApproach || to_end <= kJunctionZone) continue;

        const std::string my_base = base_road(road->id);
        const Road* my_next =
            (v->route_index + 1 < static_cast<int>(v->route.size()))
                ? v->route[v->route_index + 1]
                : nullptr;
        bool yield = false;
        Vehicle* blocker = nullptr;

        // (a) the box is taken by someone off another street
        auto occ = occupants.find(node);
        if (occ != occupants.end()) {
            for (const auto& [j, via] : occ->second) {
                if (j == i) continue;
                if (base_road(via->id) == my_base) continue;  // same street
                if (via == my_next) continue;   // already on the road I want
                yield = true;

                // A wait-for edge only when the occupant is genuinely parked.
                // Waiting behind someone who is driving away is not a deadlock,
                // and recording it as one would make the cycle search fire on
                // ordinary traffic.
                if (vehicles_[j]->current_speed < kStoppedSpeed) 
                {
                    blocker = vehicles_[j];
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
        if (!yield) {
            const double t_me =
                to_end / std::max(v->current_speed, kZoneMinSpeed) - bonus;
            auto app = approaching.find(node);
            if (app != approaching.end()) {
                for (const auto& [d, j] : app->second) {
                    if (j == i) continue;
                    const Vehicle* w = vehicles_[j];
                    if (w->current_speed < kMovingSpeed) continue;  // parked
                    if (!w->current_road) continue;
                    if (base_road(w->current_road->id) == my_base) continue;
                    const double t_w =
                        d / std::max(w->current_speed, kZoneMinSpeed);
                    if (t_w > kZoneHorizon) continue;  // far out in time
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
        }

        if (yield) {
            // Brake to the stop line rather than freeze: the line is handed to
            // IDM as the nearest obstacle when leaders are resolved.
            const double stop_gap =
                to_end - kJunctionZone - v->length_meters() * 0.5;
            zone_stop_gap_[v] = std::max(0.05, stop_gap);
            if (blocker) zone_blocker_[v] = blocker;
            v->add_zone_wait(dt);
            yields_++;
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
        Vehicle* v = vehicles_[leader];
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
    for (Vehicle* v : vehicles_) {
        if (v->stuck_time <= kStuckRescue) continue;
        if (v->get_route_index() + 1 >= static_cast<int>(v->get_route().size())) continue;
        v->rescue_to(static_cast<std::size_t>(v->get_route_index()) + 1);
        rescues_++;
    }
}
