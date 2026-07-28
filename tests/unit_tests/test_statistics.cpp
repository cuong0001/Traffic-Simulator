
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "third_party/doctest/doctest.h"
#include "utils/StatisticsManager.hpp"

TEST_CASE("StatisticsManager records and calculates metrics correctly") {
    StatisticsManager stats;

    // Test 1: Travel metrics
    stats.recordTripCompleted(100.0, 1000.0, "car");
    stats.recordTripCompleted(200.0, 2000.0, "bus");

    CHECK(stats.completedTrips() == 2);
    CHECK(stats.totalTravelTime() == 300.0);
    // (100 + 200) / 2 = 150
    CHECK(stats.averageTravelTime() == doctest::Approx(150.0)); 
    // (1000 + 2000) / 2 = 1500
    CHECK(stats.averageRouteLength() == doctest::Approx(1500.0)); 

    // Test 2: Algorithm metrics
    stats.recordRouteComputation("Dijkstra", 150.0, 45, true);
    stats.recordRouteComputation("Dijkstra", 250.0, 55, true);

    CHECK(stats.totalRecalculations() == 2);
    
    auto algo_stats = stats.algorithmStats();
    REQUIRE(algo_stats.find("Dijkstra") != algo_stats.end());
    
    CHECK(algo_stats.at("Dijkstra").computations == 2);
    CHECK(algo_stats.at("Dijkstra").failures == 0);
    CHECK(algo_stats.at("Dijkstra").avgMicros() == doctest::Approx(200.0));
    CHECK(algo_stats.at("Dijkstra").avgNodes() == doctest::Approx(50.0));
}