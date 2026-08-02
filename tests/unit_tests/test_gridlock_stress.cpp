#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "third_party/doctest/doctest.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "core/Graph.hpp"
#include "core/Intersection.hpp"
#include "core/Road.hpp"
#include "simulation/TrafficSimulator.hpp"
#include "vehicles/Vehicle.hpp"

namespace {

const char* kJunctionMap = R"({
  "type": "FeatureCollection",
  "features": [
    {"type":"Feature","properties":{"@id":"way/N_in","highway":"residential","lanes":"2","maxspeed":"50","name":"N_in"},
     "geometry":{"type":"LineString","coordinates":[[0.001,0.001],[0.001,-0.001]]}},
    {"type":"Feature","properties":{"@id":"way/S_in","highway":"residential","lanes":"2","maxspeed":"50","name":"S_in"},
     "geometry":{"type":"LineString","coordinates":[[0.001,-0.003],[0.001,-0.001]]}},
    {"type":"Feature","properties":{"@id":"way/W_in","highway":"residential","lanes":"2","maxspeed":"50","name":"W_in"},
     "geometry":{"type":"LineString","coordinates":[[-0.001,-0.001],[0.001,-0.001]]}},
    {"type":"Feature","properties":{"@id":"way/E_in","highway":"residential","lanes":"2","maxspeed":"50","name":"E_in"},
     "geometry":{"type":"LineString","coordinates":[[0.003,-0.001],[0.001,-0.001]]}},

    {"type":"Feature","properties":{"@id":"way/N_out","highway":"residential","lanes":"2","maxspeed":"50","name":"N_out"},
     "geometry":{"type":"LineString","coordinates":[[0.001,-0.001],[0.001,0.001]]}},
    {"type":"Feature","properties":{"@id":"way/S_out","highway":"residential","lanes":"2","maxspeed":"50","name":"S_out"},
     "geometry":{"type":"LineString","coordinates":[[0.001,-0.001],[0.001,-0.003]]}},
    {"type":"Feature","properties":{"@id":"way/W_out","highway":"residential","lanes":"2","maxspeed":"50","name":"W_out"},
     "geometry":{"type":"LineString","coordinates":[[0.001,-0.001],[-0.001,-0.001]]}},
    {"type":"Feature","properties":{"@id":"way/E_out","highway":"residential","lanes":"2","maxspeed":"50","name":"E_out"},
     "geometry":{"type":"LineString","coordinates":[[0.001,-0.001],[0.003,-0.001]]}}
  ]
})";

std::string write_junction_map() {
    const std::filesystem::path p =
        std::filesystem::temp_directory_path() / "uts_gridlock_junction.geojson";
    std::ofstream out(p);
    out << kJunctionMap;
    out.close();
    return p.string();
}

}  // namespace

TEST_CASE("4-way heavy traffic junction resilience under continuous influx") {
    TrafficSimulator& sim = TrafficSimulator::instance();
    const std::string map_path = write_junction_map();
    sim.load(map_path);
    sim.set_auto_spawn(false);
    sim.set_random_events(false);
    sim.set_paused(false);

    std::vector<Intersection*> outer;
    for (const auto& [name, node] : sim.graph()->intersections) {
        if (node->incoming_edges.size() + node->outgoing_edges.size() >= 2) {
            outer.push_back(node.get());
        }
    }
    REQUIRE(outer.size() >= 4);

    // Spawn opposing pairs continuously into the 4-way crossing
    int total_spawned = 0;
    for (int step = 0; step < 1000; ++step) {
        if (step % 10 == 0) {
            for (std::size_t i = 0; i < outer.size(); ++i) {
                std::size_t dest_idx = (i + 2) % outer.size();
                const std::string res = sim.spawn_vehicle(outer[i]->name, outer[dest_idx]->name, "car");
                if (!res.empty()) {
                    total_spawned++;
                }
            }
        }
        sim.tick(0.05);  // 50 seconds total simulation time
    }

    CHECK(total_spawned >= 20);

    // Verify maximum stuck time inside junction box
    int max_stuck_in_box = 0;
    for (const auto& [rid, road] : sim.graph()->edges) {
        for (Vehicle* v : road->vehicles) {
            if (!v) continue;
            const double to_end = road->length - v->position_on_road;
            if (to_end <= 6.0 || v->position_on_road <= 6.0) {
                if (v->stuck_time > max_stuck_in_box) {
                    max_stuck_in_box = static_cast<int>(v->stuck_time);
                }
            }
        }
    }

    CHECK(max_stuck_in_box <= 8);
}

TEST_CASE("Junction box does not stall crossing streams when one exit is backed up") {
    TrafficSimulator& sim = TrafficSimulator::instance();
    const std::string map_path = write_junction_map();
    sim.load(map_path);
    sim.set_auto_spawn(false);
    sim.set_random_events(false);
    sim.set_paused(false);

    Intersection* n1 = sim.graph()->intersections["n1"].get();
    Intersection* n0 = sim.graph()->intersections["n0"].get();
    Intersection* n2 = sim.graph()->intersections["n2"].get();
    Intersection* n3 = sim.graph()->intersections["n3"].get();
    REQUIRE(n1 != nullptr);
    REQUIRE(n0 != nullptr);
    REQUIRE(n2 != nullptr);
    REQUIRE(n3 != nullptr);

    // Close or block n0 exit road
    sim.trigger_event("closure", "way/N_out", 100.0);

    // Spawn vehicles from n2 -> n0 (blocked exit) AND n3 -> n2 (crossing stream)
    int cross_arrived = 0;
    for (int step = 0; step < 600; ++step) {
        if (step % 20 == 0) {
            sim.spawn_vehicle(n2->name, n0->name, "car");
            sim.spawn_vehicle(n3->name, n2->name, "car");
        }
        sim.tick(0.05);
    }

    // Crossing traffic from n3 -> n2 should still be able to flow past the junction
    for (const auto& [rid, road] : sim.graph()->edges) {
        for (Vehicle* v : road->vehicles) {
            if (v && v->get_destination() == n2) {
                if (v->position_on_road > 10.0) cross_arrived++;
            }
        }
    }

    CHECK(cross_arrived > 0);
}
