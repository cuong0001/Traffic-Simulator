#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "third_party/doctest/doctest.h"

#include <memory>
#include <vector>

#include "core/Graph.hpp"
#include "core/Intersection.hpp"
#include "core/Road.hpp"
#include "vehicles/VehicleTypes.hpp"

namespace {

// A --R1--> B --R2--> C, plus a detour A --R3--> D --R4--> C.
struct TestNet {
    TrafficGraph graph;

    TestNet() {
        add_node("A", 0.0, 0.0);
        add_node("B", 100.0, 0.0);
        add_node("C", 200.0, 0.0);
        add_node("D", 100.0, 100.0);

        add_road("R1", "A", "B", 2);
        add_road("R2", "B", "C", 2);
        add_road("R3", "A", "D", 1);
        add_road("R4", "D", "C", 1);
    }

    void add_node(const std::string& name, double x, double y) {
        graph.intersections[name] = std::make_unique<Intersection>(name, x, y);
    }

    void add_road(const std::string& id, const std::string& from,
                  const std::string& to, int lanes) {
        Intersection* a = graph.intersections.at(from).get();
        Intersection* b = graph.intersections.at(to).get();
        graph.edges[id] = std::make_unique<Road>(id, a, b, lanes, 13.9, 12, true, 100.0, nullptr);
    }

    Intersection* node(const std::string& name) { return graph.intersections.at(name).get(); }
    Road* road(const std::string& id) { return graph.edges.at(id).get(); }
};

TrafficNotification make(TrafficNotification::Kind kind, const std::string& road_id) {
    return {kind, road_id, "", "test"};
}

}  // namespace

TEST_CASE("routing strategy is selectable per vehicle") {
    TestNet net;
    for (const char* name : {"dijkstra", "astar", "bfs"}) {
        auto v = VehicleFactory::create("car", name);
        v->set_algorithm(name);
        CHECK(v->algorithm() == name);
        v->request_route(&net.graph, net.node("A"), net.node("C"));
        // Every strategy has to find *a* route between A and C.
        CHECK_FALSE(v->get_route().empty());
        CHECK(v->get_route().front()->source->name == "A");
        CHECK(v->get_route().back()->target->name == "C");
    }
}

TEST_CASE("swapping the strategy swaps the object behind PathFinding") {
    TestNet net;
    auto v = VehicleFactory::create("car", "v1");

    // A default vehicle already has a strategy installed, so routing works
    // before anyone calls set_algorithm().
    CHECK(v->algorithm() == "dijkstra");
    v->request_route(&net.graph, net.node("A"), net.node("C"));
    CHECK_FALSE(v->get_route().empty());

    // Swapping mid-life keeps the vehicle usable, and the new strategy is the
    // one actually consulted on the next route request.
    v->set_algorithm("astar");
    CHECK(v->algorithm() == "astar");
    v->recompute_route(&net.graph, net.node("C"));
    CHECK_FALSE(v->get_route().empty());

    // An unknown name must not leave the vehicle without a strategy.
    v->set_algorithm("nonsense");
    v->request_route(&net.graph, net.node("A"), net.node("C"));
    CHECK_FALSE(v->get_route().empty());
}

TEST_CASE("zone_wait is capped so a long wait cannot run away") {
    auto v = VehicleFactory::create("car", "v1");
    for (int i = 0; i < 500; ++i) v->add_zone_wait(1.0);
    CHECK(v->zone_wait() == doctest::Approx(15.0));
}

TEST_CASE("an unreachable destination leaves the vehicle without a route") {
    TrafficGraph graph;
    graph.intersections["A"] = std::make_unique<Intersection>("A", 0.0, 0.0);
    graph.intersections["B"] = std::make_unique<Intersection>("B", 100.0, 0.0);  // no road between

    auto v = VehicleFactory::create("car", "v1");
    v->request_route(&graph, graph.intersections["A"].get(), graph.intersections["B"].get());
    CHECK(v->get_route().empty());
    CHECK(v->get_current_road() == nullptr);
}

TEST_CASE("onNotify only flags a reroute when the event can reach us") {
    TestNet net;
    auto v = VehicleFactory::create("car", "v1");
    v->request_route(&net.graph, net.node("A"), net.node("C"));
    REQUIRE_FALSE(v->get_route().empty());

    SUBCASE("a closure on a road we still have to drive flags a reroute") {
        REQUIRE(v->remaining_route_contains("R1"));
        v->onNotify(make(TrafficNotification::Kind::RoadClosed, "R1"));
        CHECK(v->consume_reroute_flag());
        CHECK_FALSE(v->consume_reroute_flag());  // the flag is cleared on read
    }

    SUBCASE("a closure on a road we never touch is ignored") {
        REQUIRE_FALSE(v->remaining_route_contains("R3"));
        v->onNotify(make(TrafficNotification::Kind::RoadClosed, "R3"));
        CHECK_FALSE(v->consume_reroute_flag());
    }

    SUBCASE("a road reopening elsewhere is worth a look") {
        v->onNotify(make(TrafficNotification::Kind::RoadReopened, "R3"));
        CHECK(v->consume_reroute_flag());
    }

    SUBCASE("a road reopening on our own route changes nothing") {
        v->onNotify(make(TrafficNotification::Kind::RoadReopened, "R1"));
        CHECK_FALSE(v->consume_reroute_flag());
    }

    SUBCASE("signal events never trigger a reroute") {
        v->onNotify(make(TrafficNotification::Kind::TrafficLight, ""));
        v->onNotify(make(TrafficNotification::Kind::TrafficLightRestored, ""));
        CHECK_FALSE(v->consume_reroute_flag());
    }

    SUBCASE("request_reroute() forces one regardless") {
        v->request_reroute();
        CHECK(v->consume_reroute_flag());
    }
}

TEST_CASE("advance() never moves a vehicle through the one in front") {
    TestNet net;
    Road* r = net.road("R1");

    auto leader = VehicleFactory::create("car", "leader");
    auto follower = VehicleFactory::create("car", "follower");

    leader->current_road = r;
    leader->position_on_road = 60.0;
    r->add_vehicle(leader.get());

    follower->current_road = r;
    follower->position_on_road = 0.0;
    follower->current_speed = 14.0;
    r->add_vehicle(follower.get());

    double worst = 1e9;
    for (int i = 0; i < 400; ++i) {
        leader->current_speed = 0.0;  // a wall
        const double gap = leader->position_on_road - follower->position_on_road
                         - leader->length_meters();
        follower->advance(0.1, leader.get(), gap, i * 0.1);
        worst = std::min(worst, leader->position_on_road
                              - follower->position_on_road
                              - leader->length_meters());
    }
    CHECK(worst >= 0.0);
}

TEST_CASE("advance() settles to the tighter standstill gap") {
    TestNet net;
    Road* r = net.road("R1");

    auto leader = VehicleFactory::create("car", "leader");
    auto follower = VehicleFactory::create("car", "follower");

    leader->current_road = r;
    leader->position_on_road = 60.0;
    r->add_vehicle(leader.get());

    follower->current_road = r;
    follower->position_on_road = 0.0;
    follower->current_speed = 14.0;
    r->add_vehicle(follower.get());

    double gap = 0.0;
    for (int i = 0; i < 800; ++i) {
        leader->current_speed = 0.0;  // a wall
        gap = leader->position_on_road - follower->position_on_road
            - leader->length_meters();
        follower->advance(0.1, leader.get(), gap, i * 0.1);
    }
    // Pins the retuned standstill gap (kIdmS0 = 0.5 m, was 1.0 m): the follower
    // must be essentially stopped and settled comfortably below the old value,
    // so a regression back toward 1.0 m fails this bound.
    CHECK(follower->get_current_speed() < 0.2);
    CHECK(gap > 0.2);
    CHECK(gap < 0.9);
}

TEST_CASE("idm_accel settles to the tighter moving headway") {
    // Exercises kIdmT directly via the public static formula, rather than
    // through a Road/Vehicle whose speed cap could itself bind and mask the
    // headway term. v0 is set far out of reach so (v/v0)^4 stays negligible
    // and the fixed point is governed by the gap term alone.
    const double v0 = 100.0;
    const double v_lead = 15.0;
    double v = 15.0;
    double gap = 3.0;  // starts below the eventual equilibrium
    const double dt = 0.05;
    for (int i = 0; i < 2000; ++i) {
        const double a = Vehicle::idm_accel(v, v0, v_lead, gap);
        v = std::max(0.0, v + a * dt);
        gap += (v_lead - v) * dt;
    }
    // Pins the retuned headway (kIdmS0 + v*kIdmT = 0.5 + 15*0.15 = 2.75 m).
    // The old constants (1.0 + 15*0.25 = 4.75 m) fall well outside this
    // range, so a regression is caught.
    CHECK(gap > 2.4);
    CHECK(gap < 3.2);
}

TEST_CASE("advance() hands over to the next road and records the trip") {
    TestNet net;
    auto v = VehicleFactory::create("car", "v1");
    v->request_route(&net.graph, net.node("A"), net.node("C"));
    REQUIRE(v->get_route().size() == 2);
    REQUIRE(v->get_current_road()->id == "R1");
    CHECK(v->travel_time() == doctest::Approx(0.0));

    for (int i = 0; i < 2000 && !v->has_reached_destination(); ++i) {
        v->advance(0.1, nullptr, 1e9, i * 0.1);
    }
    CHECK(v->has_reached_destination());
    CHECK(v->travel_time() > 0.0);
    CHECK(v->get_travel_history().size() == 2);
    CHECK(v->get_travel_history().front() == "R1");
    CHECK(v->get_travel_history().back() == "R2");
}

TEST_CASE("advance() refuses to enter a slot another vehicle already holds") {
    TestNet net;
    auto blocker = VehicleFactory::create("car", "blocker");
    blocker->current_road = net.road("R2");
    blocker->position_on_road = 0.5;   // sitting right at the start of R2
    net.road("R2")->add_vehicle(blocker.get());

    auto v = VehicleFactory::create("car", "v1");
    v->request_route(&net.graph, net.node("A"), net.node("C"));
    v->position_on_road = net.road("R1")->length - 0.01;
    v->current_speed = 10.0;

    v->advance(0.1, nullptr, 1e9, 0.0);

    // It must stay on R1, parked at the stop line, instead of landing on top
    // of the blocker.
    CHECK(v->get_current_road()->id == "R1");
    CHECK(v->get_position_on_road() == doctest::Approx(net.road("R1")->length));
    CHECK(v->get_current_speed() == doctest::Approx(0.0));
}

TEST_CASE("advance() refuses a handover the old length check would have allowed") {
    TestNet net;
    // A car stopped a few metres into R2. The old entry_slot_free() only
    // weighed the STOPPED car's own length here (4.5 + 0.25 = 4.75m needed);
    // a much longer bus handing over from R1 would have passed that bar and
    // landed with its rear end still inside the car.
    auto blocker = VehicleFactory::create("car", "blocker");
    blocker->current_road = net.road("R2");
    blocker->position_on_road = 6.0;
    net.road("R2")->add_vehicle(blocker.get());

    auto v = VehicleFactory::create("bus", "v1");
    v->request_route(&net.graph, net.node("A"), net.node("C"));
    v->position_on_road = net.road("R1")->length - 0.01;
    v->current_speed = 10.0;

    v->advance(0.1, nullptr, 1e9, 0.0);

    // Must be held at R1's stop line, not overlapping the blocker.
    CHECK(v->get_current_road()->id == "R1");
    CHECK(v->get_position_on_road() == doctest::Approx(net.road("R1")->length));
    CHECK(v->get_current_speed() == doctest::Approx(0.0));
}

TEST_CASE("set_lane records the lane we came from and starts a cooldown") {
    TestNet net;
    auto v = VehicleFactory::create("car", "v1");
    v->current_road = net.road("R1");

    CHECK(v->lane == 0);
    CHECK(v->prev_lane() == -1);
    CHECK(v->lane_change_ready(0.0));

    v->set_lane(1, 100.0);
    CHECK(v->lane == 1);
    CHECK(v->prev_lane() == 0);
    CHECK(v->prev_lane_blocked_until() > 100.0);
    CHECK_FALSE(v->lane_change_ready(100.0));  // cooling down

    // The cooldown burns off as the vehicle drives.
    for (int i = 0; i < 60; ++i) v->advance(0.1, nullptr, 1e9, 100.0 + i * 0.1);
    CHECK(v->lane_change_ready(106.0));
}

TEST_CASE("forced clearance expires on its own") {
    auto v = VehicleFactory::create("car", "v1");
    CHECK_FALSE(v->is_forced(0.0));
    v->grant_forced_clearance(18.0);
    CHECK(v->is_forced(10.0));
    CHECK_FALSE(v->is_forced(18.0));
    CHECK_FALSE(v->is_forced(25.0));
}

TEST_CASE("zone_wait accumulates and clears when the vehicle gets moving") {
    TestNet net;
    auto v = VehicleFactory::create("car", "v1");
    v->request_route(&net.graph, net.node("A"), net.node("C"));

    v->add_zone_wait(2.5);
    v->add_zone_wait(1.5);
    CHECK(v->zone_wait() == doctest::Approx(4.0));

    v->current_speed = 12.0;
    v->advance(0.1, nullptr, 1e9, 0.0);
    CHECK(v->zone_wait() == doctest::Approx(0.0));
}

TEST_CASE("rescue_to jumps along the route but not onto another vehicle") {
    TestNet net;
    auto v = VehicleFactory::create("car", "v1");
    v->request_route(&net.graph, net.node("A"), net.node("C"));
    REQUIRE(v->get_route().size() == 2);

    SUBCASE("an out of range index does nothing") {
        v->rescue_to(9);
        CHECK(v->get_route_index() == 0);
    }

    SUBCASE("an occupied target road is refused") {
        auto blocker = VehicleFactory::create("car", "blocker");
        blocker->current_road = net.road("R2");
        blocker->position_on_road = 1.0;
        net.road("R2")->add_vehicle(blocker.get());

        v->rescue_to(1);
        CHECK(v->get_route_index() == 0);
        CHECK(v->get_current_road()->id == "R1");
    }

    SUBCASE("a free target road is taken") {
        v->rescue_to(1);
        CHECK(v->get_route_index() == 1);
        CHECK(v->get_current_road()->id == "R2");
        CHECK(v->get_position_on_road() == doctest::Approx(0.0));
        CHECK(v->get_current_speed() == doctest::Approx(0.0));
    }
}

TEST_CASE("congestion_sensitivity changes how much a jam slows a vehicle") {
    TestNet net;
    Road* r = net.road("R1");

    auto car = VehicleFactory::create("car", "same-id");
    auto amb = VehicleFactory::create("emergency", "same-id");  // same speed jitter

    r->set_congestion_level(0.0);
    const double car_free = car->desired_speed_on(*r);
    const double amb_free = amb->desired_speed_on(*r);

    r->set_congestion_level(1.0);
    const double car_jam = car->desired_speed_on(*r);
    const double amb_jam = amb->desired_speed_on(*r);

    CHECK(car_jam < car_free);
    CHECK(amb_jam < amb_free);
    // The ambulance filters through, so it loses a smaller fraction of speed.
    CHECK(amb_jam / amb_free > car_jam / car_free);
}
