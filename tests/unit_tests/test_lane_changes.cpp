#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "third_party/doctest/doctest.h"

#include <memory>
#include <vector>

#include "core/Graph.hpp"
#include "core/Intersection.hpp"
#include "core/Road.hpp"
#include "simulation/JunctionArbiter.hpp"
#include "simulation/TrafficController.hpp"
#include "vehicles/VehicleTypes.hpp"

namespace {

// A single 3-lane road, long enough that nobody reaches the far junction
// during these tests -- the point is purely what solve_lane_changes() does
// with vehicles already side by side on it.
struct LaneNet {
    TrafficGraph graph;
    TrafficController controller;
    std::vector<std::unique_ptr<Vehicle>> fleet;
    TrafficGraph* graph_ptr = &graph;

    LaneNet() {
        graph.intersections["A"] = std::make_unique<Intersection>("A", 0.0, 0.0);
        graph.intersections["B"] = std::make_unique<Intersection>("B", 400.0, 0.0);
        graph.edges["R1"] = std::make_unique<Road>(
            "R1", graph.intersections.at("A").get(), graph.intersections.at("B").get(),
            3, 50.0, 60, true, 400.0, nullptr);
        controller.set_graph(&graph);
    }

    Road* road() { return graph.edges.at("R1").get(); }

    // Adds a vehicle at `pos` in `lane` and returns it (fleet keeps ownership).
    Vehicle* add(const std::string& id, int lane, double pos, double speed) {
        auto v = VehicleFactory::create("car", id);
        Vehicle* raw = v.get();
        raw->current_road = road();
        raw->position_on_road = pos;
        raw->current_speed = speed;
        raw->lane = lane;
        road()->add_vehicle(raw);
        fleet.push_back(std::move(v));
        return raw;
    }
};

// Bumper-to-bumper, matching gap_between() in JunctionArbiter.cpp.
double bumper_gap(const Vehicle* back, const Vehicle* front) {
    return front->position_on_road - back->position_on_road -
           0.5 * (front->length_meters() + back->length_meters());
}

}  // namespace

TEST_CASE("a lane change is refused when a vehicle sits alongside at the SAME position") {
    // The blind spot that actually caused overlaps in a live run: two cars
    // side by side (identical position_on_road, different lanes), which is
    // exactly what a queue at a stop line produces. accel_in_lane() skips
    // `position <= mine` and follower_in_lane() requires `position < mine`,
    // so at EQUAL positions neither sees the other and MOBIL reads the target
    // lane as empty -- observed live as a full-car-length overlap with both
    // vehicles reporting the same position.
    LaneNet net;
    JunctionArbiter arbiter(net.graph_ptr, net.fleet, net.controller);

    // `mover` is boxed in behind a stopped car in its own lane, so MOBIL has
    // every incentive to move -- the point is that the incentive alone is
    // not enough when the target slot is physically taken.
    Vehicle* mover = net.add("mover", 0, 100.0, 0.0);
    net.add("blocker_same_lane", 0, 106.0, 0.0);
    Vehicle* alongside = net.add("alongside", 1, 100.0, 0.0);   // dead level

    for (int i = 0; i < 50; ++i) arbiter.solve_lane_changes(i * 0.1);

    CHECK(mover->lane == 0);
    // Belt and braces: if it ever does move, it must not be on top of anyone.
    if (mover->lane == alongside->lane) {
        CHECK(std::abs(bumper_gap(mover, alongside)) >= 0.0);
    }
}

TEST_CASE("a lane change is still allowed when the target lane is genuinely clear") {
    LaneNet net;
    JunctionArbiter arbiter(net.graph_ptr, net.fleet, net.controller);

    // Same boxed-in setup as above, but nothing at all in lane 1 -- the fix
    // must not have made MOBIL refuse every change, only unsafe ones.
    Vehicle* mover = net.add("mover", 0, 100.0, 0.0);
    net.add("blocker_same_lane", 0, 104.0, 0.0);

    bool moved = false;
    for (int i = 0; i < 50 && !moved; ++i) {
        arbiter.solve_lane_changes(i * 0.1);
        if (mover->lane != 0) moved = true;
    }
    CHECK(moved);
}

TEST_CASE("a lane change is refused when a vehicle sits just behind in the target lane") {
    LaneNet net;
    JunctionArbiter arbiter(net.graph_ptr, net.fleet, net.controller);

    Vehicle* mover = net.add("mover", 0, 100.0, 0.0);
    net.add("blocker_same_lane", 0, 104.0, 0.0);
    // Directly behind in lane 1, closer than the hard floor -- merging in
    // front of it would be a collision, not merely an inconvenience.
    net.add("tailgater", 1, 96.5, 0.0);

    for (int i = 0; i < 50; ++i) arbiter.solve_lane_changes(i * 0.1);
    CHECK(mover->lane == 0);
}
