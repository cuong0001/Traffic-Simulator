#include "simulation/MapEditor.hpp"

#include <algorithm>
#include <cmath>

#include "core/Building.hpp"
#include "core/Graph.hpp"
#include "core/Intersection.hpp"
#include "core/Road.hpp"
#include "simulation/TrafficController.hpp"
#include "vehicles/Vehicle.hpp"

MapEditor::MapEditor(TrafficGraph*& graph, std::vector<std::unique_ptr<Vehicle>>& vehicles,
                     TrafficController& controller, int& next_id)
    : graph_(graph), vehicles_(vehicles), controller_(controller), next_id_(next_id) {}

void MapEditor::reindex() {
    node_ids_.clear();
    road_ids_.clear();
    if (!graph_) return;
    for (const auto& [id, node] : graph_->intersections) node_ids_.push_back(id);
    for (const auto& [id, road] : graph_->edges) road_ids_.push_back(id);
    // Live map edits (add/split/remove a junction or road) create or drop
    // Intersection/Road objects after the loader's initial reindex(). Without
    // this, a newly created node/road keeps its default index 0 and silently
    // aliases whatever already owns that slot in JunctionArbiter's per-node
    // buckets.
    graph_->reindex();
}

void MapEditor::invalidate_routes() {
    // The graph itself changed under their wheels. This is not one of the
    // TrafficNotification kinds (those describe events on an existing network),
    // so ask each vehicle directly instead of inventing a Kind for it.
    for (auto& v : vehicles_) v->request_reroute();
}

void MapEditor::destroy_vehicle(Vehicle* v) {
    if (!v) return;
    if (v->current_road) v->current_road->remove_vehicle(v);
    controller_.detach(v);
}

// Drop a road and its reverse twin, detaching any vehicles on them.
bool MapEditor::remove_road_locked(const std::string& id) {
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
                destroy_vehicle(vit->get());
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

std::string MapEditor::add_junction_at(double x, double y) {
    if (!graph_) return "";
    const std::string id = "j" + std::to_string(next_id_++);
    graph_->intersections[id] = std::make_unique<Intersection>(id, x, y);
    reindex();
    return id;
}

std::string MapEditor::split_road_near(double x, double y) {
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
    const int lanes = best->get_lanes();
    const double speed = best->get_speed_limit();
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
            destroy_vehicle(d.v);
            vehicles_.erase(std::remove_if(vehicles_.begin(), vehicles_.end(),
                                           [&](const std::unique_ptr<Vehicle>& p) { return p.get() == d.v; }),
                            vehicles_.end());
            continue;
        }
        d.v->transfer_to_road(best_fit, best_pos);
    }

    reindex();
    invalidate_routes();
    return mid_id;
}

std::string MapEditor::add_road_between(const std::string& from,
                                        const std::string& to,
                                        double speed_kmh, int lanes,
                                        bool oneway) {
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
    reindex();
    invalidate_routes();
    return id;
}

bool MapEditor::remove_road(const std::string& id) {
    if (!remove_road_locked(id)) return false;
    reindex();
    invalidate_routes();
    return true;
}

bool MapEditor::remove_intersection(const std::string& id) {
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
    reindex();
    invalidate_routes();
    return true;
}

bool MapEditor::update_road(const std::string& id, double speed_kmh, bool closed) {
    if (!graph_) return false;
    bool touched = false;
    // Apply to both directions of a two-way street so they stay consistent.
    for (const std::string& rid : {id, id + "_r"}) {
        auto it = graph_->edges.find(rid);
        if (it == graph_->edges.end()) continue;
        if (speed_kmh > 0.0) it->second->set_speed_limit(speed_kmh);
        it->second->set_closed(closed);
        it->second->update();
        touched = true;
    }
    if (touched) invalidate_routes();
    return touched;
}

bool MapEditor::set_light(const std::string& node_id, bool on, double green) {
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

std::string MapEditor::add_building_at(double x, double y,
                                       const std::string& kind,
                                       const std::string& name,
                                       double size) {
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

bool MapEditor::remove_building(const std::string& name) {
    if (!graph_) return false;
    auto it = graph_->buildings.find(name);
    if (it == graph_->buildings.end()) return false;
    graph_->buildings.erase(it);  // unique_ptr destructor frees the Building
    return true;
}
