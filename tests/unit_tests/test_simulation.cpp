#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "third_party/doctest/doctest.h"

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "core/Graph.hpp"
#include "core/Intersection.hpp"
#include "core/Road.hpp"
#include "simulation/TrafficSimulator.hpp"
#include "vehicles/Vehicle.hpp"

namespace {

// A four-node grid written to a temp file, because TrafficSimulator loads from
// disk. Small enough that a test runs in milliseconds, connected enough that
// every node can reach every other one.
//
//   A --- B
//   |     |
//   D --- C
//
const char* kTinyMap = R"({
  "type": "FeatureCollection",
  "features": [
    {"type":"Feature","properties":{"@id":"way/1","highway":"residential","lanes":"2","maxspeed":"50","name":"North"},
     "geometry":{"type":"LineString","coordinates":[[0.0,0.0],[0.002,0.0]]}},
    {"type":"Feature","properties":{"@id":"way/2","highway":"residential","lanes":"2","maxspeed":"50","name":"East"},
     "geometry":{"type":"LineString","coordinates":[[0.002,0.0],[0.002,-0.002]]}},
    {"type":"Feature","properties":{"@id":"way/3","highway":"residential","lanes":"2","maxspeed":"50","name":"South"},
     "geometry":{"type":"LineString","coordinates":[[0.002,-0.002],[0.0,-0.002]]}},
    {"type":"Feature","properties":{"@id":"way/4","highway":"residential","lanes":"2","maxspeed":"50","name":"West"},
     "geometry":{"type":"LineString","coordinates":[[0.0,-0.002],[0.0,0.0]]}}
  ]
})";

std::string write_tiny_map() {
    const std::filesystem::path p =
        std::filesystem::temp_directory_path() / "uts_tiny_map.geojson";
    std::ofstream out(p);
    out << kTinyMap;
    out.close();
    return p.string();
}

/// The simulator is a singleton, so every test starts by reloading the map,
/// which resets vehicles, events, statistics and the clock.
TrafficSimulator& fresh() {
    static const std::string path = write_tiny_map();
    TrafficSimulator& sim = TrafficSimulator::instance();
    sim.load(path);
    sim.set_auto_spawn(false);
    sim.set_random_events(false);
    sim.set_paused(false);
    sim.set_speed(1.0);
    return sim;
}

std::vector<std::string> node_ids(TrafficSimulator& sim) {
    std::vector<std::string> ids;
    for (const auto& [id, node] : sim.graph()->intersections) ids.push_back(id);
    return ids;
}

int count_vehicles(TrafficSimulator& sim) {
    int n = 0;
    for (const auto& [id, road] : sim.graph()->edges) {
        n += static_cast<int>(road->vehicles.size());
    }
    return n;
}

/// The invariant that matters: no two vehicles in the same lane on the same
/// road may ever occupy the same metres of tarmac.
double worst_gap(TrafficSimulator& sim) {
    double worst = 1e9;
    for (const auto& [rid, road] : sim.graph()->edges) {
        const auto& vs = road->vehicles;
        for (std::size_t a = 0; a < vs.size(); ++a) {
            for (std::size_t b = a + 1; b < vs.size(); ++b) {
                if (!vs[a] || !vs[b] || vs[a]->lane != vs[b]->lane) continue;
                Vehicle* back = vs[a]->position_on_road <= vs[b]->position_on_road ? vs[a] : vs[b];
                Vehicle* front = back == vs[a] ? vs[b] : vs[a];
                worst = std::min(worst, front->position_on_road
                                      - back->position_on_road
                                      - 0.5 * (front->length_meters() + back->length_meters()));
            }
        }
    }
    return worst;
}

bool json_has(const std::string& s, const std::string& key) {
    return s.find("\"" + key + "\":") != std::string::npos;
}

}  // namespace

TEST_CASE("load builds a graph and resets the clock") {
    TrafficSimulator& sim = fresh();
    REQUIRE(sim.graph() != nullptr);
    CHECK(sim.graph()->intersections.size() >= 4);
    CHECK_FALSE(sim.graph()->edges.empty());
    CHECK(sim.state_json().find("\"time\":0") != std::string::npos);
}

TEST_CASE("spawn_vehicle rejects the pairs it cannot serve") {
    TrafficSimulator& sim = fresh();
    const auto ids = node_ids(sim);
    REQUIRE(ids.size() >= 2);

    SUBCASE("a valid pair spawns") {
        CHECK_FALSE(sim.spawn_vehicle(ids[0], ids[1], "car").empty());
        CHECK(count_vehicles(sim) == 1);
    }
    SUBCASE("origin == destination is refused") {
        CHECK(sim.spawn_vehicle(ids[0], ids[0], "car").empty());
    }
    SUBCASE("an unknown node is refused") {
        CHECK(sim.spawn_vehicle("nope", ids[1], "car").empty());
        CHECK(sim.spawn_vehicle(ids[0], "nope", "car").empty());
    }
    SUBCASE("the kerb is not double-booked") {
        // The first vehicle sits at position 0 of its first road, so a second
        // trip from the same node has nowhere to go until it moves off.
        REQUIRE_FALSE(sim.spawn_vehicle(ids[0], ids[1], "car").empty());
        CHECK(sim.spawn_vehicle(ids[0], ids[1], "car").empty());
        CHECK(count_vehicles(sim) == 1);
    }
}

TEST_CASE("the factory is used, so every vehicle type can be spawned") {
    TrafficSimulator& sim = fresh();
    const auto ids = node_ids(sim);
    for (const char* type : {"car", "bus", "emergency", "motorbike"}) {
        TrafficSimulator& s = fresh();
        const std::string id = s.spawn_vehicle(ids[0], ids[1], type);
        CHECK_FALSE(id.empty());
        CHECK(s.vehicle_detail_json(id).find(type) != std::string::npos);
    }
}

TEST_CASE("tick advances the clock, and pause and speed control it") {
    TrafficSimulator& sim = fresh();

    for (int i = 0; i < 10; ++i) sim.tick(0.1);
    CHECK(sim.state_json().find("\"time\":1") != std::string::npos);

    SUBCASE("paused freezes the clock") {
        sim.set_paused(true);
        const std::string before = sim.state_json();
        for (int i = 0; i < 10; ++i) sim.tick(0.1);
        CHECK(sim.state_json().substr(0, 20) == before.substr(0, 20));
    }
    SUBCASE("speed scales dt") {
        sim.set_speed(4.0);
        for (int i = 0; i < 10; ++i) sim.tick(0.1);   // 1 s wall -> 4 s sim
        CHECK(sim.state_json().find("\"time\":5") != std::string::npos);
    }
}

TEST_CASE("events are created, described and expire on their own") {
    TrafficSimulator& sim = fresh();
    const std::string road = sim.graph()->edges.begin()->first;

    SUBCASE("an unknown type creates nothing") {
        CHECK(sim.trigger_event("banana", road, 10.0).empty());
    }
    SUBCASE("a closure applies, shows up in state, and undoes itself") {
        const std::string id = sim.trigger_event("closure", road, 5.0);
        REQUIRE_FALSE(id.empty());
        CHECK(sim.graph()->get_edge(road)->is_closed());
        CHECK(sim.state_json().find(id) != std::string::npos);

        for (int i = 0; i < 60; ++i) sim.tick(0.1);   // 6 s > 5 s duration
        CHECK_FALSE(sim.graph()->get_edge(road)->is_closed());
        CHECK(sim.state_json().find("\"id\":\"" + id + "\"") == std::string::npos);
    }
    SUBCASE("a road cannot be closed twice") {
        REQUIRE_FALSE(sim.trigger_event("closure", road, 30.0).empty());
        CHECK(sim.trigger_event("closure", road, 30.0).empty());
    }
    SUBCASE("an accident restores capacity when it clears") {
        Road* r = sim.graph()->get_edge(road);
        const double before = r->get_capacity_scale();
        REQUIRE_FALSE(sim.trigger_event("accident", road, 3.0).empty());
        CHECK(r->get_capacity_scale() < before);
        for (int i = 0; i < 40; ++i) sim.tick(0.1);
        CHECK(r->get_capacity_scale() == doctest::Approx(before));
    }
}

TEST_CASE("closing a road on a vehicle's route makes it replan") {
    TrafficSimulator& sim = fresh();
    const auto ids = node_ids(sim);
    const std::string vid = sim.spawn_vehicle(ids[0], ids[2], "car");
    REQUIRE_FALSE(vid.empty());

    // Find a road the vehicle still has to drive, and close it.
    std::string on_route;
    for (const auto& [rid, road] : sim.graph()->edges) {
        for (Vehicle* v : road->vehicles) {
            if (v && v->id == vid && !v->get_route().empty()) {
                on_route = v->get_route().back()->id;
            }
        }
    }
    REQUIRE_FALSE(on_route.empty());

    const std::string before = sim.vehicle_detail_json(vid);
    REQUIRE_FALSE(sim.trigger_event("closure", on_route, 30.0).empty());
    sim.tick(0.1);   // the reroute happens on the tick after the notification
    // Either the route changed, or there was no alternative -- both are fine,
    // what matters is that the vehicle survived the event.
    CHECK_FALSE(sim.vehicle_detail_json(vid).empty());
    CHECK(sim.vehicle_detail_json(vid).find(vid) != std::string::npos);
}

TEST_CASE("no two vehicles ever share a slot in the same lane") {
    TrafficSimulator& sim = fresh();
    sim.set_auto_spawn(true);
    sim.set_spawn_rate(8.0);
    sim.set_target_vehicles(20);

    double worst = 1e9;
    for (int i = 0; i < 900; ++i) {   // 30 simulated seconds at 1/30 s
        sim.tick(1.0 / 30.0);
        worst = std::min(worst, worst_gap(sim));
    }
    CHECK(worst >= 0.0);
}

TEST_CASE("the vehicle target is respected") {
    TrafficSimulator& sim = fresh();
    sim.set_auto_spawn(true);
    sim.set_spawn_rate(50.0);      // far faster than the cap allows
    sim.set_target_vehicles(6);

    for (int i = 0; i < 600; ++i) sim.tick(1.0 / 30.0);
    CHECK(count_vehicles(sim) <= 6);
}

TEST_CASE("vehicle operations") {
    TrafficSimulator& sim = fresh();
    const auto ids = node_ids(sim);
    const std::string vid = sim.spawn_vehicle(ids[0], ids[1], "car");
    REQUIRE_FALSE(vid.empty());

    SUBCASE("detail json names the vehicle") {
        CHECK(sim.vehicle_detail_json(vid).find(vid) != std::string::npos);
        // An unknown id answers with an error object, not an empty one, so the
        // UI can tell "gone" apart from "no data yet".
        CHECK(sim.vehicle_detail_json("ghost").find("error") != std::string::npos);
    }
    SUBCASE("removing works once") {
        CHECK(sim.remove_vehicle(vid));
        CHECK_FALSE(sim.remove_vehicle(vid));
        CHECK(count_vehicles(sim) == 0);
    }
    SUBCASE("clear_vehicles reports how many it removed") {
        CHECK(sim.clear_vehicles() == 1);
        CHECK(count_vehicles(sim) == 0);
    }
    SUBCASE("per-vehicle algorithm is validated") {
        CHECK(sim.set_vehicle_algorithm(vid, "astar"));
        CHECK(sim.set_vehicle_algorithm(vid, "bfs"));
        CHECK_FALSE(sim.set_vehicle_algorithm(vid, "banana"));
        CHECK_FALSE(sim.set_vehicle_algorithm("ghost", "astar"));
    }
}

TEST_CASE("the default routing strategy is validated") {
    TrafficSimulator& sim = fresh();
    CHECK(sim.set_algorithm("astar"));
    CHECK(sim.set_algorithm("bfs"));
    CHECK(sim.set_algorithm("dijkstra"));
    CHECK_FALSE(sim.set_algorithm("banana"));
}

TEST_CASE("live map editing") {
    TrafficSimulator& sim = fresh();
    const std::size_t nodes_before = sim.graph()->intersections.size();
    const std::size_t roads_before = sim.graph()->edges.size();

    SUBCASE("a junction can be added and joined to the network") {
        Intersection* any = sim.graph()->intersections.begin()->second.get();
        const std::string nid = sim.add_junction_at(any->x + 40.0, any->y + 40.0);
        REQUIRE_FALSE(nid.empty());
        CHECK(sim.graph()->intersections.size() == nodes_before + 1);

        const std::string rid = sim.add_road_between(any->name, nid, 50.0, 2, false);
        CHECK_FALSE(rid.empty());
        CHECK(sim.graph()->edges.size() > roads_before);
    }
    SUBCASE("live edits keep Intersection::index and Road::index collision-free") {
        // Before the fix, a node/road created after the initial load kept the
        // default index 0 and silently aliased whatever already owned that
        // slot in solve_junctions()'s per-node buckets. Guard against a
        // regression directly, without needing to simulate traffic across the
        // new junction to observe the corruption.
        Intersection* any = sim.graph()->intersections.begin()->second.get();
        const std::string nid = sim.add_junction_at(any->x + 40.0, any->y + 40.0);
        REQUIRE_FALSE(nid.empty());
        const std::string rid = sim.add_road_between(any->name, nid, 50.0, 2, false);
        REQUIRE_FALSE(rid.empty());

        std::set<uint32_t> node_indices;
        for (const auto& [id, node] : sim.graph()->intersections) {
            CHECK(node_indices.insert(node->index).second);  // no duplicate index
        }
        std::set<uint32_t> road_indices;
        for (const auto& [id, road] : sim.graph()->edges) {
            CHECK(road_indices.insert(road->index).second);  // no duplicate index
        }

        // base_index legitimately repeats -- it groups a two-way street's "id"
        // and "id_r" twin -- but the new road must not fall into some
        // unrelated pre-existing street's group.
        Road* new_road = sim.graph()->get_edge(rid);
        REQUIRE(new_road != nullptr);
        for (const auto& [id, road] : sim.graph()->edges) {
            if (road.get() == new_road || id == rid) continue;
            if (id == rid + "_r") continue;
            CHECK(road->base_index != new_road->base_index);
        }
    }
    SUBCASE("removing a road takes its reverse twin with it") {
        const std::string rid = sim.graph()->edges.begin()->first;
        CHECK(sim.remove_road(rid));
        CHECK(sim.graph()->get_edge(rid) == nullptr);
        CHECK_FALSE(sim.remove_road(rid));
    }
    SUBCASE("a road's speed and closed flag can be edited") {
        const std::string rid = sim.graph()->edges.begin()->first;
        CHECK(sim.update_road(rid, 30.0, true));
        CHECK(sim.graph()->get_edge(rid)->is_closed());
        CHECK(sim.update_road(rid, 60.0, false));
        CHECK_FALSE(sim.graph()->get_edge(rid)->is_closed());
        CHECK_FALSE(sim.update_road("ghost", 30.0, false));
    }
}

TEST_CASE("map files: save, list, reload, and reject path traversal") {
    TrafficSimulator& sim = fresh();

    SUBCASE("a name with a separator or .. is refused") {
        CHECK_FALSE(sim.save_map("../escape"));
        CHECK_FALSE(sim.save_map("sub/dir"));
        CHECK_FALSE(sim.save_map("back\\slash"));
        CHECK_FALSE(sim.load_map("../../etc/passwd"));
    }
    SUBCASE("a saved map appears in the listing and reloads") {
        const std::size_t roads = sim.graph()->edges.size();
        REQUIRE(sim.save_map("uts_test_roundtrip"));
        CHECK(sim.maps_json().find("uts_test_roundtrip") != std::string::npos);
        REQUIRE(sim.load_map("uts_test_roundtrip"));
        CHECK(sim.graph()->edges.size() == roads);
        std::filesystem::remove("data/maps/uts_test_roundtrip.geojson");
    }
}

TEST_CASE("state_json carries everything the UI needs") {
    TrafficSimulator& sim = fresh();
    const auto ids = node_ids(sim);
    sim.spawn_vehicle(ids[0], ids[1], "car");
    sim.trigger_event("congestion", sim.graph()->edges.begin()->first, 20.0);
    for (int i = 0; i < 90; ++i) sim.tick(1.0 / 30.0);

    const std::string s = sim.state_json();
    for (const char* key : {"time", "paused", "speed", "vehicles", "lights",
                            "roads", "events", "log", "solver", "active", "stats"}) {
        CAPTURE(key);
        CHECK(json_has(s, key));
    }
    CHECK(s.front() == '{');
    CHECK(s.back() == '}');
    // The event log is fed by the controller, so a triggered event shows up.
    CHECK(s.find("Heavy congestion") != std::string::npos);
}

TEST_CASE("map_json is served and is non-trivial") {
    TrafficSimulator& sim = fresh();
    const std::string m = sim.map_json();
    CHECK(m.size() > 100);
    CHECK(m.front() == '{');
    CHECK(m.back() == '}');
}

TEST_CASE("the wait-for graph tells a deadlock from a queue") {
    // A queue: 0 waits on 1 waits on 2, and 2 waits on nobody. Every vehicle
    // here is behaving correctly -- the one at the front will move eventually
    // and the rest follow. The 10-second stopwatch this replaced could not see
    // that, and grabbed all three.
    SUBCASE("a chain is not a deadlock, however long anyone has waited") {
        const std::vector<int> waiting_on{1, 2, -1};
        const std::vector<double> stuck{90.0, 90.0, 90.0};
        CHECK(find_gridlock_cycle_leaders(waiting_on, stuck).empty());
    }

    // A ring: each of the four waits on the next, and the fourth waits on the
    // first. Nobody will ever move. This is a deadlock at 3 seconds just as
    // much as at 300, which is exactly why time is the wrong test.
    SUBCASE("a four-vehicle ring is found, and broken at its longest sufferer") {
        const std::vector<int> waiting_on{1, 2, 3, 0};
        const std::vector<double> stuck{4.0, 31.0, 7.0, 2.0};
        const auto leaders = find_gridlock_cycle_leaders(waiting_on, stuck);
        REQUIRE(leaders.size() == 1);
        // One grant is enough: the vehicle that moves frees the one behind it,
        // and so on around the ring. Pick the one that has suffered longest.
        CHECK(leaders[0] == 1);
    }

    SUBCASE("two separate rings are both found") {
        const std::vector<int> waiting_on{1, 0, 3, 4, 2};
        const std::vector<double> stuck{5.0, 1.0, 1.0, 9.0, 1.0};
        const auto leaders = find_gridlock_cycle_leaders(waiting_on, stuck);
        REQUIRE(leaders.size() == 2);
        CHECK(leaders[0] == 0);   // ring {0,1}, worst is 0
        CHECK(leaders[1] == 3);   // ring {2,3,4}, worst is 3
    }

    SUBCASE("a chain feeding into a ring reports the ring once, not twice") {
        // 0 -> 1 -> 2 -> 3 -> 1: the tail is a queue behind a genuine deadlock.
        // The ring must be reported, and reported once.
        const std::vector<int> waiting_on{1, 2, 3, 1};
        const std::vector<double> stuck{50.0, 2.0, 8.0, 3.0};
        const auto leaders = find_gridlock_cycle_leaders(waiting_on, stuck);
        REQUIRE(leaders.size() == 1);
        // Vehicle 0 has waited longest but is NOT in the ring; releasing it
        // would achieve nothing, because what blocks it is the ring ahead.
        CHECK(leaders[0] == 2);
    }

    SUBCASE("a vehicle waiting on itself is a deadlock of one") {
        const std::vector<int> waiting_on{-1, 1};
        const std::vector<double> stuck{0.0, 12.0};
        const auto leaders = find_gridlock_cycle_leaders(waiting_on, stuck);
        REQUIRE(leaders.size() == 1);
        CHECK(leaders[0] == 1);
    }

    SUBCASE("nobody waiting on anybody is not a deadlock") {
        const std::vector<int> waiting_on{-1, -1, -1};
        const std::vector<double> stuck{0.0, 0.0, 0.0};
        CHECK(find_gridlock_cycle_leaders(waiting_on, stuck).empty());
    }
}

TEST_CASE("splitting a road under a vehicle keeps the vehicle") {
    TrafficSimulator& sim = fresh();
    sim.set_target_vehicles(0);

    const auto ids = node_ids(sim);
    REQUIRE(ids.size() >= 4);
    const std::string id = sim.spawn_vehicle(ids[0], ids[2], "car");
    REQUIRE(!id.empty());

    // Let it get well onto its first road.
    for (int i = 0; i < 200; ++i) sim.tick(0.05);

    Vehicle* before = nullptr;
    for (const auto& [rid, road] : sim.graph()->edges) {
        for (Vehicle* v : road->vehicles) {
            if (v && v->id == id) before = v;
        }
    }
    REQUIRE(before != nullptr);
    REQUIRE(before->current_road != nullptr);

    // Where it is in the world, and how fast, before the edit.
    Road* road = before->current_road;
    const double t = before->position_on_road / road->length;
    const double wx = road->source->x + (road->target->x - road->source->x) * t;
    const double wy = road->source->y + (road->target->y - road->source->y) * t;
    const double speed_before = before->current_speed;

    // Split the road right where the vehicle is standing.
    const std::string mid = sim.split_road_near(wx, wy);
    REQUIRE(!mid.empty());

    // The vehicle must still exist. Before PR-10 it was despawned along with
    // the road it happened to be on, so editing the map deleted traffic.
    Vehicle* after = nullptr;
    for (const auto& [rid, r] : sim.graph()->edges) {
        for (Vehicle* v : r->vehicles) {
            if (v && v->id == id) after = v;
        }
    }
    REQUIRE(after != nullptr);
    REQUIRE(after->current_road != nullptr);

    // And it must be in the same place in the world, not teleported to the
    // start of some road.
    Road* nr = after->current_road;
    const double nt = after->position_on_road / nr->length;
    const double nx = nr->source->x + (nr->target->x - nr->source->x) * nt;
    const double ny = nr->source->y + (nr->target->y - nr->source->y) * nt;
    CHECK(std::hypot(nx - wx, ny - wy) < 2.0);      // metres

    // Motion state survives too: the driver should not feel the edit.
    CHECK(after->current_speed == doctest::Approx(speed_before));

    // The old route ran along a road that no longer exists, so it must have
    // been reduced and flagged for replanning rather than left dangling.
    CHECK(after->get_route().size() == 1);
    CHECK(after->get_route()[0] == nr);

    // The simulator keeps running afterwards without tripping over it.
    for (int i = 0; i < 200; ++i) sim.tick(0.05);
    CHECK(worst_gap(sim) > -0.5);
}

TEST_CASE("a vehicle reports why it is not moving") {
    TrafficSimulator& sim = fresh();
    sim.set_target_vehicles(0);

    const auto ids = node_ids(sim);
    const std::string id = sim.spawn_vehicle(ids[0], ids[2], "car");
    REQUIRE(!id.empty());

    for (int i = 0; i < 100; ++i) sim.tick(0.05);

    // state is reported per vehicle so the UI can say "waiting at a light"
    // instead of showing a stopped car and leaving the viewer to guess.
    const std::string state = sim.state_json();
    CHECK(state.find("\"state\":") != std::string::npos);

    Vehicle* v = nullptr;
    for (const auto& [rid, road] : sim.graph()->edges) {
        for (Vehicle* w : road->vehicles) {
            if (w && w->id == id) v = w;
        }
    }
    REQUIRE(v != nullptr);
    // On an empty map with no signals it should simply be driving.
    CHECK(v->state() == Vehicle::State::Moving);
}

// ===========================================================================
// M6: how far out a driver starts working out whether to give way
// ===========================================================================
namespace {

// A dense grid: 0.00025 degrees is about 28 m, which is what a real
// OpenStreetMap extract looks like. kTinyMap's blocks are ~222 m, closer to the
// hand-drawn downtown map the constant 26 m was written for. The whole point of
// deriving the window from the map is that these two need different answers.
const char* kDenseMap = R"({
  "type": "FeatureCollection",
  "features": [
    {"type":"Feature","properties":{"@id":"way/1","highway":"residential","lanes":"2"},
     "geometry":{"type":"LineString","coordinates":[[0.0,0.0],[0.00025,0.0],[0.0005,0.0],[0.00075,0.0]]}},
    {"type":"Feature","properties":{"@id":"way/2","highway":"residential","lanes":"2"},
     "geometry":{"type":"LineString","coordinates":[[0.0,-0.00025],[0.00025,-0.00025],[0.0005,-0.00025],[0.00075,-0.00025]]}},
    {"type":"Feature","properties":{"@id":"way/3","highway":"residential","lanes":"2"},
     "geometry":{"type":"LineString","coordinates":[[0.0,0.0],[0.0,-0.00025]]}},
    {"type":"Feature","properties":{"@id":"way/4","highway":"residential","lanes":"2"},
     "geometry":{"type":"LineString","coordinates":[[0.00025,0.0],[0.00025,-0.00025]]}},
    {"type":"Feature","properties":{"@id":"way/5","highway":"residential","lanes":"2"},
     "geometry":{"type":"LineString","coordinates":[[0.0005,0.0],[0.0005,-0.00025]]}},
    {"type":"Feature","properties":{"@id":"way/6","highway":"residential","lanes":"2"},
     "geometry":{"type":"LineString","coordinates":[[0.00075,0.0],[0.00075,-0.00025]]}}
  ]
})";

// A roundabout: a four-segment one-way ring tagged junction=roundabout, with
// two approach roads meeting it. This is the shape OSM actually delivers -- the
// ring is never one closed way, it is several short ones, so the joins between
// them look like ordinary crossings unless something says otherwise.
const char* kRoundaboutMap = R"({
  "type": "FeatureCollection",
  "features": [
    {"type":"Feature","properties":{"@id":"way/10","highway":"residential","junction":"roundabout","lanes":"1"},
     "geometry":{"type":"LineString","coordinates":[[0.0,0.00012],[0.00012,0.0]]}},
    {"type":"Feature","properties":{"@id":"way/11","highway":"residential","junction":"roundabout","lanes":"1"},
     "geometry":{"type":"LineString","coordinates":[[0.00012,0.0],[0.0,-0.00012]]}},
    {"type":"Feature","properties":{"@id":"way/12","highway":"residential","junction":"roundabout","lanes":"1"},
     "geometry":{"type":"LineString","coordinates":[[0.0,-0.00012],[-0.00012,0.0]]}},
    {"type":"Feature","properties":{"@id":"way/13","highway":"residential","junction":"roundabout","lanes":"1"},
     "geometry":{"type":"LineString","coordinates":[[-0.00012,0.0],[0.0,0.00012]]}},
    {"type":"Feature","properties":{"@id":"way/20","highway":"residential","lanes":"2"},
     "geometry":{"type":"LineString","coordinates":[[0.0,0.0012],[0.0,0.00012]]}},
    {"type":"Feature","properties":{"@id":"way/21","highway":"residential","lanes":"2"},
     "geometry":{"type":"LineString","coordinates":[[0.0,-0.00012],[0.0,-0.0012]]}}
  ]
})";

std::string write_map(const char* text, const char* filename) {
    const std::filesystem::path p = std::filesystem::temp_directory_path() / filename;
    std::ofstream out(p);
    out << text;
    out.close();
    return p.string();
}

}  // namespace

TEST_CASE("the give-way distance comes from the map's own scale") {
    TrafficSimulator& sim = TrafficSimulator::instance();

    // 222 m blocks: 26 m out is a genuine final approach, so keep the value the
    // reference uses. Half a block would be 111 m, hence the ceiling.
    static const std::string tiny = write_map(kTinyMap, "uts_scale_tiny.geojson");
    sim.load(tiny);
    CHECK(sim.junction_approach() == doctest::Approx(26.0));

    // 28 m blocks: 26 m is the whole street. A driver that starts negotiating
    // right of way the instant it turns into a road never gets a free metre,
    // and on this map that was 51% of the fleet at once.
    static const std::string dense = write_map(kDenseMap, "uts_scale_dense.geojson");
    sim.load(dense);
    CHECK(sim.junction_approach() < 26.0);
    CHECK(sim.junction_approach() > 6.0);   // still clears the junction box

    SUBCASE("and it can be overridden live, within sane limits") {
        sim.set_junction_approach(12.0);
        CHECK(sim.junction_approach() == doctest::Approx(12.0));
        sim.set_junction_approach(0.0);       // clamped up: below the box is meaningless
        CHECK(sim.junction_approach() >= 3.0);
        sim.set_junction_approach(5000.0);    // clamped down
        CHECK(sim.junction_approach() <= 60.0);
    }

    SUBCASE("loading a map re-derives it, discarding a hand-set value") {
        // Deliberate: carrying a 11 m window onto a map with 222 m blocks would
        // have drivers deciding almost inside the junction, with nothing on
        // screen to explain it.
        sim.set_junction_approach(11.0);
        CHECK(sim.junction_approach() == doctest::Approx(11.0));
        sim.load(tiny);
        CHECK(sim.junction_approach() == doctest::Approx(26.0));
    }
}

TEST_CASE("a roundabout is not a ring of crossings") {
    TrafficSimulator& sim = TrafficSimulator::instance();
    static const std::string path = write_map(kRoundaboutMap, "uts_roundabout.geojson");
    sim.load(path);
    sim.set_auto_spawn(false);
    sim.set_random_events(false);
    sim.set_paused(false);

    int circulating = 0, ordinary = 0;
    for (const auto& [id, road] : sim.graph()->edges) {
        if (road->is_internal) circulating++; else ordinary++;
    }
    // The four ring ways survive as circulating roads; the approaches are
    // two-way, so they contribute their "_r" twins as well.
    CHECK(circulating >= 4);
    CHECK(ordinary >= 2);

    SUBCASE("the seams between ring segments are not arbitration points") {
        // A node with nothing but circulating roads on it is inside the ring:
        // no stream crosses there, so nobody gives way to anybody. Where an
        // approach meets the ring, the node stays an ordinary crossing -- that
        // is what makes entering traffic give way.
        int inside = 0, entries = 0;
        for (const auto& [id, node] : sim.graph()->intersections) {
            bool has_ring = false, has_street = false;
            for (const auto& [rid, road] : sim.graph()->edges) {
                if (road->source != node.get() && road->target != node.get()) continue;
                if (road->is_internal) has_ring = true; else has_street = true;
            }
            if (!has_ring) continue;
            if (has_street) { entries++; CHECK_FALSE(node->is_roundabout); }
            else            { inside++;  CHECK(node->is_roundabout); }
        }
        // Both kinds must exist, or the checks above prove nothing.
        CHECK(inside > 0);
        CHECK(entries > 0);
    }

    SUBCASE("cars going round it never give way to each other") {
        // The whole complaint, stated as a number. With ONLY circulating
        // traffic on the map, no vehicle has anything to give way to: every
        // conflict it could possibly find is another car going round the same
        // ring. The seams between segments are not crossings at all, and at an
        // entry node -- which IS a crossing, because an approach meets it --
        // two circulating cars still have no quarrel with each other.
        //
        // So the yield counter must stay at exactly zero. Before roundabouts
        // were recorded as such, each of those seams arbitrated like an
        // ordinary junction and the ring gave way to itself all the way round.
        sim.set_target_vehicles(0);

        // Ring nodes only: no approach traffic, so nothing legitimately yields.
        std::vector<std::string> ring_nodes;
        for (const auto& [id, node] : sim.graph()->intersections) {
            for (const auto& [rid, road] : sim.graph()->edges) {
                if (!road->is_internal) continue;
                if (road->source == node.get() || road->target == node.get()) {
                    ring_nodes.push_back(id);
                    break;
                }
            }
        }
        REQUIRE(ring_nodes.size() >= 3);

        // Pack the ring. A handful of cars spread round 190 m of tarmac never
        // meet at the same node, and a test that never creates the conflict
        // cannot show the conflict is handled.
        int spawned = 0;
        for (int round = 0; round < 8; ++round) {
            for (std::size_t a = 0; a < ring_nodes.size(); ++a) {
                for (std::size_t b = 0; b < ring_nodes.size(); ++b) {
                    if (a == b) continue;
                    if (!sim.spawn_vehicle(ring_nodes[a], ring_nodes[b], "car").empty()) {
                        spawned++;
                    }
                }
            }
            for (int t = 0; t < 12; ++t) sim.tick(0.05);
        }
        REQUIRE(spawned >= 8);
        CHECK(count_vehicles(sim) >= 6);

        for (int i = 0; i < 600; ++i) sim.tick(0.05);
        const std::string state = sim.state_json();
        const std::size_t at = state.find("\"yields\":");
        REQUIRE(at != std::string::npos);
        const long yields = std::strtol(state.c_str() + at + 9, nullptr, 10);
        CHECK(yields == 0);
    }

    SUBCASE("traffic goes round it instead of seizing up") {
        // Fill the ring and the approaches. If circulating vehicles gave way to
        // each other at every seam -- which is what happens when the ring is
        // read as a chain of crossings -- this deadlocks and nothing moves.
        sim.set_target_vehicles(0);
        const auto ids = node_ids(sim);
        int spawned = 0;
        for (std::size_t a = 0; a < ids.size() && spawned < 12; ++a) {
            for (std::size_t b = 0; b < ids.size() && spawned < 12; ++b) {
                if (a == b) continue;
                if (!sim.spawn_vehicle(ids[a], ids[b], "car").empty()) spawned++;
            }
        }
        REQUIRE(spawned > 0);

        double fastest = 0.0;
        for (int i = 0; i < 400; ++i) {
            sim.tick(0.05);
            for (const auto& [rid, road] : sim.graph()->edges) {
                for (Vehicle* v : road->vehicles) {
                    if (v) fastest = std::max(fastest, v->get_current_speed());
                }
            }
        }
        CHECK(fastest > 1.0);          // something is actually driving
        CHECK(worst_gap(sim) > -0.5);  // and not by overlapping each other
    }

    SUBCASE("saving and reloading keeps the roundabout a roundabout") {
        REQUIRE(sim.save_map("uts_roundabout_saved"));
        REQUIRE(sim.load_map("uts_roundabout_saved"));
        int still_circulating = 0;
        for (const auto& [id, road] : sim.graph()->edges) {
            if (road->is_internal) still_circulating++;
        }
        CHECK(still_circulating >= 4);
        std::filesystem::remove("data/maps/uts_roundabout_saved.geojson");
    }
}

TEST_CASE("a rescue is only counted when it happens") {
    TrafficSimulator& sim = fresh();
    sim.set_target_vehicles(0);

    const auto ids = node_ids(sim);
    const std::string id = sim.spawn_vehicle(ids[0], ids[2], "car");
    REQUIRE(!id.empty());

    Vehicle* v = nullptr;
    for (const auto& [rid, road] : sim.graph()->edges) {
        for (Vehicle* w : road->vehicles) if (w && w->id == id) v = w;
    }
    REQUIRE(v != nullptr);
    REQUIRE(v->get_route().size() >= 2);

    // Jumping to the next road of the route works, and says so.
    CHECK(v->rescue_to(static_cast<std::size_t>(v->get_route_index()) + 1));

    // Past the end of the route there is nothing to jump to. Before the fix the
    // caller counted a rescue for this too -- and because a refusal leaves
    // stuck_time alone on purpose, a permanently wedged vehicle booked one
    // every single tick: 19212 of them in 9 sim-seconds on small_map.
    CHECK_FALSE(v->rescue_to(v->get_route().size()));
    CHECK_FALSE(v->rescue_to(v->get_route().size() + 5));
}

TEST_CASE("landing on top of somebody is refused, and not counted") {
    // The refusal that actually mattered. rescue_to() also gives up when the
    // slot it would land in is occupied, and THAT is the one a wedged vehicle
    // hits every tick forever -- the route-index checks above fire once and
    // then the vehicle is finished anyway.
    //
    // On a grid, not the 4-node ring: routing is congestion-aware, so two cars
    // leaving the same corner can go opposite ways round a ring and share no
    // road at all. A grid gives overlapping routes to work with.
    TrafficSimulator& sim = TrafficSimulator::instance();
    static const std::string dense = write_map(kDenseMap, "uts_rescue_dense.geojson");
    sim.load(dense);
    sim.set_auto_spawn(false);
    sim.set_random_events(false);
    sim.set_paused(false);
    sim.set_target_vehicles(0);

    const auto ids = node_ids(sim);
    REQUIRE(ids.size() >= 4);

    auto find = [&](const std::string& id) -> Vehicle* {
        for (const auto& [rid, road] : sim.graph()->edges) {
            for (Vehicle* w : road->vehicles) if (w && w->id == id) return w;
        }
        return nullptr;
    };

    // A handful of vehicles, spaced out in time so each one can leave the
    // start slot before the next arrives.
    std::vector<Vehicle*> fleet;
    for (std::size_t k = 0; k + 1 < ids.size() && fleet.size() < 8; ++k) {
        const std::string id = sim.spawn_vehicle(ids[k], ids[ids.size() - 1 - k], "car");
        if (!id.empty()) {
            Vehicle* v = find(id);
            if (v && v->get_route().size() > 1) fleet.push_back(v);
        }
        for (int t = 0; t < 20; ++t) sim.tick(0.05);
    }
    REQUIRE(fleet.size() >= 2);

    // Two of them that still have a road in common ahead of them.
    Vehicle* a = nullptr;
    Vehicle* b = nullptr;
    std::size_t ak = 0, bm = 0;
    for (std::size_t x = 0; x < fleet.size() && !a; ++x) {
        for (std::size_t y = 0; y < fleet.size() && !a; ++y) {
            if (x == y) continue;
            const std::vector<Road*>& ar = fleet[x]->get_route();
            const std::vector<Road*>& br = fleet[y]->get_route();
            const std::size_t a0 = static_cast<std::size_t>(std::max(0, fleet[x]->get_route_index())) + 1;
            const std::size_t b0 = static_cast<std::size_t>(std::max(0, fleet[y]->get_route_index()));
            for (std::size_t k = a0; k < ar.size() && !a; ++k) {
                for (std::size_t m = b0; m < br.size(); ++m) {
                    if (ar[k] != br[m] || !ar[k]) continue;
                    a = fleet[x]; b = fleet[y]; ak = k; bm = m;
                    break;
                }
            }
        }
    }
    REQUIRE(a != nullptr);   // no shared road: the test would prove nothing

    REQUIRE(a->rescue_to(ak));         // `a` is now at position 0 of that road
    CHECK_FALSE(b->rescue_to(bm));     // so `b` cannot materialise inside it
}

// Guards the exact bugs found while wiring /api/stateBin into the frontend
// (docs/PLAN-fast-ids-and-streaming.md, Phase 3): the type byte used to check
// for "truck"/"ambulance", which don't exist in this codebase (real types are
// car/bus/emergency/motorbike), so those two never matched; and num_id used to
// require the id to start with 'v', which spawn_locked's ids ("car5",
// "emergency9", ...) never do, so every vehicle fell into a non-reversible
// hash. Both would have silently broken the frontend's click-select and route
// highlighting, which reconstruct a vehicle's display id as
// TYPE_NAMES[type] + num_id.
TEST_CASE("encode_binary_snapshot round-trips vehicle state") {
    TrafficSimulator& sim = fresh();
    const auto ids = node_ids(sim);
    REQUIRE(ids.size() >= 2);

    auto find = [&](const std::string& id) -> Vehicle* {
        for (const auto& [rid, road] : sim.graph()->edges) {
            for (Vehicle* w : road->vehicles) if (w && w->id == id) return w;
        }
        return nullptr;
    };

    // One of each of the four types this codebase actually has, spaced out in
    // time (as the rescue test above does) so each clears the kerb before the
    // next needs the same handful of nodes on this tiny map.
    const std::vector<std::string> types = {"car", "bus", "emergency", "motorbike"};
    std::vector<std::string> spawned_ids;
    for (const auto& t : types) {
        std::string id;
        for (std::size_t k = 0; k + 1 < ids.size() && id.empty(); ++k) {
            id = sim.spawn_vehicle(ids[k], ids[k + 1], t);
        }
        REQUIRE_FALSE(id.empty());
        spawned_ids.push_back(id);
        for (int t2 = 0; t2 < 20; ++t2) sim.tick(0.05);
    }

    std::vector<uint8_t> buf;
    sim.encode_binary_snapshot(buf);

    REQUIRE(buf.size() >= sizeof(BinarySnapshotHeader));
    BinarySnapshotHeader header;
    std::memcpy(&header, buf.data(), sizeof(header));
    CHECK(header.magic == 0x54524146u);
    CHECK(header.version == 2);
    CHECK(header.vehicle_count == spawned_ids.size());
    REQUIRE(buf.size() == sizeof(BinarySnapshotHeader) +
                              static_cast<std::size_t>(header.vehicle_count) * sizeof(VehicleSnapshotBin));

    const auto* records = reinterpret_cast<const VehicleSnapshotBin*>(buf.data() + sizeof(header));
    static const char* kTypeNames[4] = {"car", "bus", "emergency", "motorbike"};

    for (const std::string& spawned_id : spawned_ids) {
        // Find it by rebuilding the display id exactly the way the frontend
        // does, so this test fails the same way a real regression would show
        // up in the browser.
        const VehicleSnapshotBin* match = nullptr;
        for (std::size_t r = 0; r < header.vehicle_count; ++r) {
            REQUIRE(records[r].type < 4);
            const std::string rebuilt =
                std::string(kTypeNames[records[r].type]) + std::to_string(records[r].num_id);
            if (rebuilt == spawned_id) { match = &records[r]; break; }
        }
        REQUIRE(match != nullptr);

        Vehicle* v = find(spawned_id);
        REQUIRE(v != nullptr);
        REQUIRE(v->current_road != nullptr);
        CHECK(v->type() == kTypeNames[match->type]);
        CHECK(match->base_index == v->current_road->base_index);
        CHECK(match->pos == doctest::Approx(v->position_on_road));
        CHECK(match->lane == v->lane);
        CHECK(((match->flags & 1) != 0) == v->current_road->reverse_twin);
    }
}

// ===========================================================================
// Spawn lane selection: multi-lane roads must not run as single-lane roads
// ===========================================================================
namespace {

// A single two-way street with 3 lanes per direction ("lanes":"6" splits
// (6+1)/2 = 3 each way) between two junctions -- wide enough that if spawn
// lane selection is biased toward index 0, it shows up immediately.
const char* kWideRoadMap = R"({
  "type": "FeatureCollection",
  "features": [
    {"type":"Feature","properties":{"@id":"way/1","highway":"primary","lanes":"6","maxspeed":"50"},
     "geometry":{"type":"LineString","coordinates":[[0.0,0.0],[0.01,0.0]]}}
  ]
})";

}  // namespace

TEST_CASE("spawning spreads across empty lanes instead of always picking lane 0") {
    TrafficSimulator& sim = TrafficSimulator::instance();
    static const std::string path = write_map(kWideRoadMap, "uts_wide_road.geojson");
    sim.load(path);
    sim.set_auto_spawn(false);
    sim.set_random_events(false);
    sim.set_paused(false);

    const auto ids = node_ids(sim);
    REQUIRE(ids.size() == 2);

    // The old rule initialised its "best gap" search at 1e9 and kept the
    // first candidate on a strict >, so on an empty road every lane tied and
    // the lowest-numbered one always won: every vehicle in the city spawned
    // in lane 0, and because MOBIL only moves a vehicle that is actually held
    // up, a 3-lane road ran as a 1-lane road forever.
    std::map<int, int> hist;
    for (int i = 0; i < 60; ++i) {
        const std::string id = sim.spawn_vehicle(ids[0], ids[1], "car");
        REQUIRE(!id.empty());
        for (const auto& [rid, road] : sim.graph()->edges) {
            for (Vehicle* v : road->vehicles) {
                if (v && v->id == id) hist[v->lane]++;
            }
        }
        // Clear before the next spawn: this test is about the LANE CHOICE at
        // spawn time, not about queueing behaviour once the road fills up.
        sim.clear_vehicles();
    }

    int lanes_used = 0;
    for (const auto& [lane, n] : hist) if (n > 0) lanes_used++;
    CAPTURE(hist[0]); CAPTURE(hist[1]); CAPTURE(hist[2]);
    CHECK(lanes_used >= 2);        // more than one lane actually got chosen
    // Fixed seed (rng_{2026}), so this is a property of the code, not a coin
    // flip on the day: no single lane should take an implausible majority of
    // 60 tries at 1/3 each.
    CHECK(hist[0] < 45);
}
