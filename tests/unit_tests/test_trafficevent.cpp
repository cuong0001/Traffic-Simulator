#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "third_party/doctest/doctest.h"

#include "core/Graph.hpp"
#include "core/Intersection.hpp"
#include "core/Road.hpp"
#include "events/TrafficEvent.hpp"

// TEST 1: Check for road closures and openings. (RoadClosureEvent)
TEST_CASE("RoadClosureEvent closes and reopens a road") {
    TrafficGraph graph;

    // 1. Initialize the intersection and add road R1 to the graph.
    auto* nodeA = new Intersection("NodeA", 0.0, 0.0);
    auto* nodeB = new Intersection("NodeB", 100.0, 0.0);
    graph.intersections["NodeA"] = nodeA;
    graph.intersections["NodeB"] = nodeB;

    // Create Road R1 (2 lanes, max speed 50.0, two-way)
    graph.create_directional_road("R1", nodeA, nodeB, 2, 50.0, false);
    Road* road = graph.get_edge("R1");
    REQUIRE(road != nullptr);

    // Record initial travel expenses.
    road->update_travel_cost();
    double original_cost = road->travel_cost;

    // 2. Trigger the road closure event.
    RoadClosureEvent closure("evt_closure", "R1", 60.0);
    closure.apply(graph);

    // Check: Road blocked and cost increased to infinity (1e8).
    CHECK(road->is_closed == true);
    CHECK(road->travel_cost == doctest::Approx(1e8));

    // 3. End of the road closure event
    closure.expire(graph);

    // Kiểm tra: Đường mở lại và chi phí khôi phục như cũ
    CHECK(road->is_closed == false);
    CHECK(road->travel_cost == doctest::Approx(original_cost));
}

// TEST 2: Check for overlapping accidents (AccidentEvent)
TEST_CASE("AccidentEvent restores capacity correctly with overlapping accidents") {
    TrafficGraph graph;

    auto* nodeA = new Intersection("NodeC", 0.0, 0.0);
    auto* nodeB = new Intersection("NodeD", 100.0, 0.0);
    graph.intersections["NodeC"] = nodeA;
    graph.intersections["NodeD"] = nodeB;

    graph.create_directional_road("R2", nodeA, nodeB, 2, 50.0, false);
    Road* road = graph.get_edge("R2");
    REQUIRE(road != nullptr);

    // Initial state
    CHECK(road->capacity_scale == doctest::Approx(1.0));
    CHECK(road->event_congestion == doctest::Approx(0.0));

    // Create two overlapping accidents on the same R2 road.
    AccidentEvent acc1("acc1", "R2", 30.0);
    AccidentEvent acc2("acc2", "R2", 45.0);

    // Accident 1 occurred: capacity_scale = 1.0 * 0.3 = 0.3
    acc1.apply(graph);
    CHECK(road->capacity_scale == doctest::Approx(0.3));

    // Accident 2 occurred:: capacity_scale = 0.3 * 0.3 = 0.09
    acc2.apply(graph);
    CHECK(road->capacity_scale == doctest::Approx(0.09));

    // Accident 1 is no longer in effect: capacity_scale = 0.09 / 0.3 = 0.3
    acc1.expire(graph);
    CHECK(road->capacity_scale == doctest::Approx(0.3));

    // Accident 2 is no longer in effect: capacity_scale = 0.3 / 0.3 = 1.0
    acc2.expire(graph);
    CHECK(road->capacity_scale == doctest::Approx(1.0));
}

// TEST 3: Check for traffic jam incidents (CongestionEvent)
TEST_CASE("CongestionEvent increases and restores event_congestion") {
    TrafficGraph graph;

    auto* nodeA = new Intersection("NodeE", 0.0, 0.0);
    auto* nodeB = new Intersection("NodeF", 100.0, 0.0);
    graph.intersections["NodeE"] = nodeA;
    graph.intersections["NodeF"] = nodeB;

    graph.create_directional_road("R3", nodeA, nodeB, 2, 50.0, false);
    Road* road = graph.get_edge("R3");
    REQUIRE(road != nullptr);

    double base_cost = road->travel_cost;

    // Create a traffic jam of significant severity intensity = 0.5
    CongestionEvent congestion("cong1", "R3", 60.0, 0.5);

    // Activate
    congestion.apply(graph);
    CHECK(road->event_congestion == doctest::Approx(0.5));
    CHECK(road->travel_cost > base_cost);

    // End
    congestion.expire(graph);
    CHECK(road->event_congestion == doctest::Approx(0.0));
    CHECK(road->travel_cost == doctest::Approx(base_cost));
}