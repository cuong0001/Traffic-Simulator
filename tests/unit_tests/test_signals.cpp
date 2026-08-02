#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "third_party/doctest/doctest.h"

#include <limits>
#include <memory>
#include <set>
#include <string>

#include "core/Graph.hpp"
#include "core/Intersection.hpp"
#include "core/Road.hpp"
#include "core/TrafficLight.hpp"

namespace {

// A plain four-way crossing centred on "M": east-west street A--M--C, and
// north-south street B--M--D. Every road is two-way, so M has four approaches
// lying on two perpendicular axes -- the case a traffic signal exists for.
//
//            B (0, 100)
//               |
//  A (-100,0) --M-- C (100, 0)
//               |
//            D (0, -100)
TrafficGraph* four_way() {
    TrafficGraph* g = new TrafficGraph();
    auto node = [&](const std::string& id, double x, double y) {
        g->intersections[id] = std::make_unique<Intersection>(id, x, y);
    };
    node("M", 0, 0);
    node("A", -100, 0);
    node("B", 0, 100);
    node("C", 100, 0);
    node("D", 0, -100);

    auto link = [&](const std::string& a, const std::string& b) {
        Intersection* u = g->intersections[a].get();
        Intersection* v = g->intersections[b].get();
        for (int dir = 0; dir < 2; ++dir) {
            Intersection* s = dir ? v : u;
            Intersection* t = dir ? u : v;
            const std::string id = dir ? (a + b + "_r") : (a + b);
            // Road's constructor already appends itself to
            // source->outgoing_edges and target->incoming_edges.
            g->create_directional_road(id, s, t, 1, 50.0, false);
        }
    };
    link("A", "M");
    link("B", "M");
    link("C", "M");
    link("D", "M");
    return g;
}

}  // namespace

TEST_CASE("a four-way crossing splits into two opposing phase groups") {
    TrafficGraph* g = four_way();
    REQUIRE(g->signalize("M"));

    Intersection* m = g->intersections["M"].get();
    REQUIRE(m->is_signalized);
    REQUIRE(m->incoming_edges.size() == 4);

    // The east-west pair must share a group, and so must the north-south pair,
    // because opposing traffic on one street does not conflict.
    const int g_am = g->edges["AM"]->signal_group;   // arriving from the west
    const int g_cm = g->edges["CM"]->signal_group;   // arriving from the east
    const int g_bm = g->edges["BM"]->signal_group;   // arriving from the north
    const int g_dm = g->edges["DM"]->signal_group;   // arriving from the south

    CHECK(g_am == g_cm);
    CHECK(g_bm == g_dm);
    CHECK(g_am != g_bm);   // the two streets must NOT share a group

    delete g;
}

TEST_CASE("opposing approaches never show green at the same moment") {
    TrafficGraph* g = four_way();
    REQUIRE(g->signalize("M"));
    Intersection* m = g->intersections["M"].get();

    bool saw_ew_green = false, saw_ns_green = false;
    for (double t = 0.0; t < 120.0; t += 0.25) {
        const unsigned int ew = m->signal_state_for(g->edges["AM"]->signal_group, t);
        const unsigned int ns = m->signal_state_for(g->edges["BM"]->signal_group, t);

        // The whole point of phase groups: never both open.
        const bool both_green = (ew == TrafficLight::Green) && (ns == TrafficLight::Green);
        CHECK_FALSE(both_green);
        if (ew == TrafficLight::Green) saw_ew_green = true;
        if (ns == TrafficLight::Green) saw_ns_green = true;
    }
    // ...and never one street starved either.
    CHECK(saw_ew_green);
    CHECK(saw_ns_green);

    delete g;
}

TEST_CASE("the cycle is a pure function of time, so it cannot drift") {
    TrafficGraph* g = four_way();
    REQUIRE(g->signalize("M"));
    Intersection* m = g->intersections["M"].get();
    const int grp = g->edges["AM"]->signal_group;

    const double cycle = (m->light_system->get_green_duration() +
                          m->light_system->get_yellow_duration() +
                          m->light_system->get_red_duration()) *
                         m->light_system->get_groups();

    for (double t = 0.0; t < cycle; t += 0.5) {
        CHECK(m->signal_state_for(grp, t) == m->signal_state_for(grp, t + cycle));
        CHECK(m->signal_state_for(grp, t) == m->signal_state_for(grp, t + 5 * cycle));
    }
    delete g;
}

TEST_CASE("two junctions do not switch in unison") {
    TrafficGraph* g = four_way();
    // A second crossing, far away, with its own four approaches on two axes.
    g->intersections["N"] = std::make_unique<Intersection>("N", 500, 500);
    const double px[4] = {400, 500, 600, 500};
    const double py[4] = {500, 600, 500, 400};
    for (int i = 0; i < 4; ++i) {
        const std::string arm = "P" + std::to_string(i);
        g->intersections[arm] = std::make_unique<Intersection>(arm, px[i], py[i]);
        g->create_directional_road(arm + "N", g->intersections[arm].get(),
                                   g->intersections["N"].get(), 1, 50.0, true);
    }
    REQUIRE(g->signalize("M"));
    REQUIRE(g->signalize("N"));

    CHECK(g->intersections["M"]->light_system->get_phase_offset() !=
          doctest::Approx(g->intersections["N"]->light_system->get_phase_offset()));
    delete g;
}

TEST_CASE("the phase offset is derived from the id, so a reload reproduces it") {
    TrafficLight a, b;
    a.set_phase_offset_from("junction-42");
    b.set_phase_offset_from("junction-42");
    CHECK(a.get_phase_offset() == doctest::Approx(b.get_phase_offset()));

    TrafficLight c;
    c.set_phase_offset_from("junction-43");
    CHECK(c.get_phase_offset() != doctest::Approx(a.get_phase_offset()));
}

TEST_CASE("an approach with no group assigned is red, never green") {
    TrafficGraph* g = four_way();
    REQUIRE(g->signalize("M"));
    Intersection* m = g->intersections["M"].get();

    // -1 is what a road carries before any group is handed out. Reading it as
    // "group 0" would wave unassigned traffic through a live junction.
    for (double t = 0.0; t < 60.0; t += 0.5) {
        CHECK(m->signal_state_for(-1, t) == TrafficLight::Red);
        CHECK(m->signal_state_for(99, t) == TrafficLight::Red);
    }
    delete g;
}

TEST_CASE("a junction with nothing to arbitrate is refused a guessed signal") {
    TrafficGraph* g = new TrafficGraph();
    g->intersections["M"] = std::make_unique<Intersection>("M", 0, 0);
    g->intersections["A"] = std::make_unique<Intersection>("A", -100, 0);
    g->intersections["C"] = std::make_unique<Intersection>("C", 100, 0);

    // A--M--C is one straight street: two approaches, a single axis.
    for (auto [id, from] : {std::pair<std::string, std::string>{"AM", "A"},
                            std::pair<std::string, std::string>{"CM", "C"}}) {
        g->create_directional_road(id, g->intersections[from].get(), g->intersections["M"].get(),
                                   1, 50.0, true);
    }

    // signalize() guesses, so it must refuse: stopping a straight street for
    // itself would be pure loss.
    CHECK_FALSE(g->signalize("M"));
    CHECK_FALSE(g->intersections["M"]->is_signalized);

    // assign_signal_groups() is what an explicit map file goes through, and it
    // still assigns -- one shared group, so nobody is stranded on red.
    CHECK(g->assign_signal_groups("M") == 1);
    CHECK(g->edges["AM"]->signal_group == 0);
    CHECK(g->edges["CM"]->signal_group == 0);

    delete g;
}

TEST_CASE("the stop line sits before the junction and inside the road") {
    TrafficGraph* g = four_way();
    REQUIRE(g->signalize("M"));

    Road* am = g->edges["AM"].get();
    CHECK(am->signal_stop_from_end == doctest::Approx(8.0));
    CHECK(am->stop_line_position() == doctest::Approx(am->length - 8.0));
    CHECK(am->stop_line_position() < am->length);
    CHECK(am->stop_line_position() > 0.0);

    // A road with no signal reports its far end, so callers need no branch.
    Road* unsignalled = g->edges["AM_r"].get();   // leaves M, so it is not an approach
    CHECK(unsignalled->signal_group == -1);
    CHECK(unsignalled->stop_line_position() == doctest::Approx(unsignalled->length));

    delete g;
}

TEST_CASE("a short block keeps its stop line off the start of the road") {
    TrafficGraph* g = new TrafficGraph();
    auto node = [&](const std::string& id, double x, double y) {
        g->intersections[id] = std::make_unique<Intersection>(id, x, y);
    };
    node("M", 0, 0);
    node("A", -10, 0);   // 10 m block, far shorter than the nominal 8 m setback
    node("B", 0, 10);
    node("C", 10, 0);

    for (auto id : {std::string("A"), std::string("B"), std::string("C")}) {
        g->create_directional_road(id + "M", g->intersections[id].get(),
                                   g->intersections["M"].get(), 1, 50.0, true);
    }
    REQUIRE(g->signalize("M"));

    Road* am = g->edges["AM"].get();
    REQUIRE(am->length == doctest::Approx(10.0));
    // 40% of the road, not a flat 8 m, so the line stays comfortably inside it.
    CHECK(am->signal_stop_from_end == doctest::Approx(4.0));
    CHECK(am->stop_line_position() == doctest::Approx(6.0));

    delete g;
}

TEST_CASE("designalize clears the groups off every approach") {
    TrafficGraph* g = four_way();
    REQUIRE(g->signalize("M"));
    REQUIRE(g->edges["AM"]->signal_group >= 0);

    g->designalize("M");
    CHECK_FALSE(g->intersections["M"]->is_signalized);
    CHECK(g->intersections["M"]->light_system == nullptr);
    for (Road* r : g->intersections["M"]->incoming_edges) {
        CHECK(r->signal_group == -1);
        // ...and an unsignalized junction is green to everyone, so removing a
        // light must not leave traffic stopped at a ghost.
        CHECK(g->intersections["M"]->signal_state_for(r->signal_group, 3.0) ==
              TrafficLight::Green);
    }
    delete g;
}

TEST_CASE("Road::cost follows the requested policy and refuses closed roads") {
    TrafficGraph* g = four_way();
    Road* am = g->edges["AM"].get();
    REQUIRE(am->length == doctest::Approx(100.0));

    CHECK(am->cost(CostMode::Distance) == doctest::Approx(100.0));

    // 50 km/h = 13.89 m/s, so 100 m takes about 7.2 s on an empty road.
    CHECK(am->cost(CostMode::Time) == doctest::Approx(100.0 / (50.0 / 3.6)).epsilon(0.01));

    // Congestion makes it slower in Time, and leaves Distance untouched.
    am->set_congestion_level(1.0);
    CHECK(am->cost(CostMode::Time) > 100.0 / (50.0 / 3.6));
    CHECK(am->cost(CostMode::Distance) == doctest::Approx(100.0));

    // A closed road is not expensive, it is impassable.
    am->set_closed(true);
    CHECK(am->cost(CostMode::Distance) == std::numeric_limits<double>::infinity());
    CHECK(am->cost(CostMode::Time) == std::numeric_limits<double>::infinity());

    delete g;
}

TEST_CASE("effective speed falls with congestion but never reaches zero") {
    TrafficGraph* g = four_way();
    Road* am = g->edges["AM"].get();

    CHECK(am->speed_limit_ms() == doctest::Approx(50.0 / 3.6));
    CHECK(am->effective_speed_ms() == doctest::Approx(am->speed_limit_ms()));

    am->set_congestion_level(1.0);
    const double jammed = am->effective_speed_ms();
    CHECK(jammed < am->speed_limit_ms());
    CHECK(jammed >= 1.0);   // a jammed road must still let traffic crawl out

    // A vehicle that ignores congestion keeps the free speed.
    CHECK(am->effective_speed_ms(0.0) == doctest::Approx(am->speed_limit_ms()));

    delete g;
}
