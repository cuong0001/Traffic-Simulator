#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "third_party/doctest/doctest.h"

#include <memory>

#include "core/Graph.hpp"
#include "core/Intersection.hpp"
#include "core/Road.hpp"
#include "events/TrafficEvent.hpp"

// TEST 1: Check for road closures and openings. (RoadClosureEvent)
TEST_CASE("RoadClosureEvent closes and reopens a road") {
    TrafficGraph graph;

    // 1. Initialize the intersection and add road R1 to the graph.
    auto nodeA_owned = std::make_unique<Intersection>("NodeA", 0.0, 0.0);
    Intersection* nodeA = nodeA_owned.get();
    auto nodeB_owned = std::make_unique<Intersection>("NodeB", 100.0, 0.0);
    Intersection* nodeB = nodeB_owned.get();
    graph.intersections["NodeA"] = std::move(nodeA_owned);
    graph.intersections["NodeB"] = std::move(nodeB_owned);

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

    // Check: Road reopening and restoration travel costs
    CHECK(road->is_closed == false);
    CHECK(road->travel_cost == doctest::Approx(original_cost));
}

// TEST 2: Check for overlapping accidents (AccidentEvent)
TEST_CASE("AccidentEvent restores capacity correctly with overlapping accidents") {
    TrafficGraph graph;

    auto nodeA_owned = std::make_unique<Intersection>("NodeC", 0.0, 0.0);
    Intersection* nodeA = nodeA_owned.get();
    auto nodeB_owned = std::make_unique<Intersection>("NodeD", 100.0, 0.0);
    Intersection* nodeB = nodeB_owned.get();
    graph.intersections["NodeC"] = std::move(nodeA_owned);
    graph.intersections["NodeD"] = std::move(nodeB_owned);

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

    auto nodeA_owned = std::make_unique<Intersection>("NodeE", 0.0, 0.0);
    Intersection* nodeA = nodeA_owned.get();
    auto nodeB_owned = std::make_unique<Intersection>("NodeF", 100.0, 0.0);
    Intersection* nodeB = nodeB_owned.get();
    graph.intersections["NodeE"] = std::move(nodeA_owned);
    graph.intersections["NodeF"] = std::move(nodeB_owned);

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

// =========================================================================
// TEST 4: Check for overlapping traffic light events. (TrafficLightEvent)
// =========================================================================
TEST_CASE("TrafficLightEvent restores green duration with overlapping events") {
    TrafficGraph graph;
    
    // 1. Initialize the NodeL intersection
    auto node_owned = std::make_unique<Intersection>("NodeL", 0.0, 0.0);
    Intersection* node = node_owned.get();

    // The initial green light setting is 30.0 seconds. (vàng 3.0s, đỏ 2.0s)
    node->light_system = std::make_unique<TrafficLight>(30.0, 3.0, 2.0);
    graph.intersections["NodeL"] = std::move(node_owned);

    // Initial state
    CHECK(node->light_system->get_green_duration() == doctest::Approx(30.0));

    // Initialize two traffic light failure events at the same intersection.
    TrafficLightEvent light1("evt_L1", "NodeL", 20.0);
    TrafficLightEvent light2("evt_L2", "NodeL", 30.0);

    // 3. Activate event 1: Green light decreases to 20% (30.0 * 0.2 = 6.0)
    light1.apply(graph);
    CHECK(node->light_system->get_green_duration() == doctest::Approx(6.0));

    // 4. Triggering the overlapping Event 2: The green light continues to decrease. (6.0 * 0.2 = 1.2)
    light2.apply(graph);
    CHECK(node->light_system->get_green_duration() == doctest::Approx(1.2));

    // 5. End Event 1: Restoration of the Event 1 segment. (1.2 / 0.2 = 6.0)
    light1.expire(graph);
    CHECK(node->light_system->get_green_duration() == doctest::Approx(6.0));

    // 6. End Event 2: Complete restoration to the original state. (6.0 / 0.2 = 30.0)
    light2.expire(graph);
    CHECK(node->light_system->get_green_duration() == doctest::Approx(30.0));
}
