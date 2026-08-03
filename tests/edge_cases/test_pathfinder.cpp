// Edge-case tests for Dijkstra, BFS & A* through the PathFinding strategy interface.
// Uses doctest (header-only, v2.4.11). Quick syntax reference:
//
//   TEST_SUITE("Group name") {         -- optional grouping
//     TEST_CASE("description") {       -- one test
//       REQUIRE(expr);                 -- hard assert (stops test on failure)
//       CHECK(expr);                   -- soft assert (continues on failure)
//       CHECK_FALSE(expr);             -- assert expr is false
//       REQUIRE_THROWS(expr);          -- assert an exception is thrown
//       INFO("x = " << x);            -- context printed only on failure
//       SUBCASE("variant") { ... }     -- inline parameterisation
//     }
//   }
//
// Build & run:
//   cmake -S . -B build
//   cmake --build build --target test_pathfinder_edge
//   ./build/Debug/test_pathfinder_edge             (Windows)
//   ./build/test_pathfinder_edge                   (Linux/Mac)
//
// Adding a new test:
//   1. Write a TEST_CASE inside the existing TEST_SUITE block.
//   2. Build a TrafficGraph in-memory (see unit_tests/test_pathfinder.cpp helpers or inline).
//   3. Call find_path() through PathFinding*, Dijkstra, or BFS.
//   4. Assert with CHECK / REQUIRE.


#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "third_party/doctest/doctest.h"

#include "core/Graph.hpp"
#include "core/Intersection.hpp"
#include "core/Road.hpp"
#include "routing/PathFinding.hpp"
#include "routing/Dijkstra.hpp"
#include "routing/BFS.hpp"
#include "routing/AStar.hpp"

#include <memory>
#include <string>
#include <vector>

static std::vector<std::string> names_of(const std::vector<Intersection*>& path) {
    std::vector<std::string> out;
    out.reserve(path.size());
    for (Intersection* n : path) out.push_back(n->name);
    return out;
}

TEST_SUITE("PathFinding – Edge Cases") {

    TEST_CASE("Non-existent start node returns empty path") {
        std::unique_ptr<TrafficGraph> g(new TrafficGraph());
        g->intersections["A"] = std::make_unique<Intersection>("A", 0, 0);

        Dijkstra dijk;
        BFS bfs;
        AStar astar;

        CHECK(dijk.find_path(*g, "GHOST", "A").empty());
        CHECK(bfs.find_path(*g, "GHOST", "A").empty());
        CHECK(astar.find_path(*g, "GHOST", "A").empty());
    }

    TEST_CASE("Non-existent end node returns empty path") {
        std::unique_ptr<TrafficGraph> g(new TrafficGraph());
        g->intersections["A"] = std::make_unique<Intersection>("A", 0, 0);

        Dijkstra dijk;
        BFS bfs;
        AStar astar;

        CHECK(dijk.find_path(*g, "A", "GHOST").empty());
        CHECK(bfs.find_path(*g, "A", "GHOST").empty());
        CHECK(astar.find_path(*g, "A", "GHOST").empty());
    }

    TEST_CASE("Both nodes non-existent returns empty path") {
        TrafficGraph g;

        Dijkstra dijk;
        BFS bfs;
        AStar astar;

        CHECK(dijk.find_path(g, "X", "Y").empty());
        CHECK(bfs.find_path(g, "X", "Y").empty());
        CHECK(astar.find_path(g, "X", "Y").empty());
    }

    TEST_CASE("Empty graph returns empty path") {
        TrafficGraph g;

        Dijkstra dijk;
        BFS bfs;
        AStar astar;

        CHECK(dijk.find_path(g, "A", "B").empty());
        CHECK(bfs.find_path(g, "A", "B").empty());
        CHECK(astar.find_path(g, "A", "B").empty());
    }

    TEST_CASE("Single node, self-path") {
        std::unique_ptr<TrafficGraph> g(new TrafficGraph());
        g->intersections["X"] = std::make_unique<Intersection>("X", 0, 0);

        Dijkstra dijk;
        auto pd = dijk.find_path(*g, "X", "X");
        REQUIRE(pd.size() == 1);
        CHECK(pd[0]->name == "X");

        AStar astar;
        auto pa = astar.find_path(*g, "X", "X");
        REQUIRE(pa.size() == 1);
        CHECK(pa[0]->name == "X");
    }

    TEST_CASE("Two disconnected components – no cross-component path") {
        std::unique_ptr<TrafficGraph> g(new TrafficGraph());
        g->intersections["A"] = std::make_unique<Intersection>("A", 0, 0);
        g->intersections["B"] = std::make_unique<Intersection>("B", 1, 0);
        g->intersections["C"] = std::make_unique<Intersection>("C", 10, 0);
        g->intersections["D"] = std::make_unique<Intersection>("D", 11, 0);

        // Component 1: A→B
        g->edges["AB"] = std::make_unique<Road>("AB", g->intersections["A"].get(), g->intersections["B"].get(), 1, 50, 12, true, 1.0);
        // Component 2: C→D
        g->edges["CD"] = std::make_unique<Road>("CD", g->intersections["C"].get(), g->intersections["D"].get(), 1, 50, 12, true, 1.0);

        Dijkstra dijk;
        BFS bfs;
        AStar astar;

        SUBCASE("Within a component succeeds") {
            CHECK_FALSE(dijk.find_path(*g, "A", "B").empty());
            CHECK_FALSE(bfs.find_path(*g, "A", "B").empty());
            CHECK_FALSE(astar.find_path(*g, "A", "B").empty());
        }

        SUBCASE("Cross-component returns empty") {
            CHECK(dijk.find_path(*g, "A", "D").empty());
            CHECK(bfs.find_path(*g, "A", "D").empty());
            CHECK(astar.find_path(*g, "A", "D").empty());
            CHECK(dijk.find_path(*g, "C", "B").empty());
            CHECK(bfs.find_path(*g, "C", "B").empty());
            CHECK(astar.find_path(*g, "C", "B").empty());
        }
    }

    TEST_CASE("Cycle – algorithms terminate and find the exit") {
        //  A → B → C → A (cycle) and C → D (exit)
        std::unique_ptr<TrafficGraph> g(new TrafficGraph());
        g->intersections["A"] = std::make_unique<Intersection>("A", 0, 0);
        g->intersections["B"] = std::make_unique<Intersection>("B", 1, 0);
        g->intersections["C"] = std::make_unique<Intersection>("C", 2, 0);
        g->intersections["D"] = std::make_unique<Intersection>("D", 3, 0);

        g->edges["AB"] = std::make_unique<Road>("AB", g->intersections["A"].get(), g->intersections["B"].get(), 1, 50, 12, true, 1.0);
        g->edges["BC"] = std::make_unique<Road>("BC", g->intersections["B"].get(), g->intersections["C"].get(), 1, 50, 12, true, 1.0);
        g->edges["CA"] = std::make_unique<Road>("CA", g->intersections["C"].get(), g->intersections["A"].get(), 1, 50, 12, true, 1.0);
        g->edges["CD"] = std::make_unique<Road>("CD", g->intersections["C"].get(), g->intersections["D"].get(), 1, 50, 12, true, 1.0);

        Dijkstra dijk;
        BFS bfs;
        AStar astar;

        auto pd = dijk.find_path(*g, "A", "D");
        auto pb = bfs.find_path(*g, "A", "D");
        auto pa = astar.find_path(*g, "A", "D");

        std::vector<std::string> expected = {"A", "B", "C", "D"};
        CHECK(names_of(pd) == expected);
        CHECK(names_of(pb) == expected);
        CHECK(names_of(pa) == expected);
    }

    TEST_CASE("Reverse direction – unreachable despite edge existing") {
        std::unique_ptr<TrafficGraph> g(new TrafficGraph());
        g->intersections["A"] = std::make_unique<Intersection>("A", 0, 0);
        g->intersections["B"] = std::make_unique<Intersection>("B", 1, 0);

        // Only A→B, not B→A
        g->edges["AB"] = std::make_unique<Road>("AB", g->intersections["A"].get(), g->intersections["B"].get(), 1, 50, 12, true, 1.0);

        Dijkstra dijk;
        BFS bfs;
        AStar astar;

        CHECK_FALSE(dijk.find_path(*g, "A", "B").empty());
        CHECK_FALSE(bfs.find_path(*g, "A", "B").empty());
        CHECK_FALSE(astar.find_path(*g, "A", "B").empty());

        CHECK(dijk.find_path(*g, "B", "A").empty());
        CHECK(bfs.find_path(*g, "B", "A").empty());
        CHECK(astar.find_path(*g, "B", "A").empty());
    }

    TEST_CASE("Two equal-weight paths – Dijkstra returns one of them") {
        //  A →(5)→ B →(5)→ D
        //  A →(5)→ C →(5)→ D
        std::unique_ptr<TrafficGraph> g(new TrafficGraph());
        g->intersections["A"] = std::make_unique<Intersection>("A", 0, 0);
        g->intersections["B"] = std::make_unique<Intersection>("B", 5, 5);
        g->intersections["C"] = std::make_unique<Intersection>("C", 5, -5);
        g->intersections["D"] = std::make_unique<Intersection>("D", 10, 0);

        g->edges["AB"] = std::make_unique<Road>("AB", g->intersections["A"].get(), g->intersections["B"].get(), 1, 50, 12, true, 5.0);
        g->edges["BD"] = std::make_unique<Road>("BD", g->intersections["B"].get(), g->intersections["D"].get(), 1, 50, 12, true, 5.0);
        g->edges["AC"] = std::make_unique<Road>("AC", g->intersections["A"].get(), g->intersections["C"].get(), 1, 50, 12, true, 5.0);
        g->edges["CD"] = std::make_unique<Road>("CD", g->intersections["C"].get(), g->intersections["D"].get(), 1, 50, 12, true, 5.0);

        Dijkstra dijk;
        auto ids_d = names_of(dijk.find_path(*g, "A", "D"));

        REQUIRE(ids_d.size() == 3);
        CHECK(ids_d.front() == "A");
        CHECK(ids_d.back()  == "D");
        CHECK((ids_d[1] == "B" || ids_d[1] == "C"));

        AStar astar;
        auto ids_a = names_of(astar.find_path(*g, "A", "D"));

        REQUIRE(ids_a.size() == 3);
        CHECK(ids_a.front() == "A");
        CHECK(ids_a.back()  == "D");
        CHECK((ids_a[1] == "B" || ids_a[1] == "C"));
    }

    TEST_CASE("Star topology – hub to leaves, no leaf-to-leaf path") {
        std::unique_ptr<TrafficGraph> g(new TrafficGraph());
        g->intersections["H"]  = std::make_unique<Intersection>("H", 0, 0);
        g->intersections["L1"] = std::make_unique<Intersection>("L1", 1, 0);
        g->intersections["L2"] = std::make_unique<Intersection>("L2", 0, 1);
        g->intersections["L3"] = std::make_unique<Intersection>("L3", -1, 0);

        g->edges["HL1"] = std::make_unique<Road>("HL1", g->intersections["H"].get(), g->intersections["L1"].get(), 1, 50, 12, true, 1.0);
        g->edges["HL2"] = std::make_unique<Road>("HL2", g->intersections["H"].get(), g->intersections["L2"].get(), 1, 50, 12, true, 1.0);
        g->edges["HL3"] = std::make_unique<Road>("HL3", g->intersections["H"].get(), g->intersections["L3"].get(), 1, 50, 12, true, 1.0);

        Dijkstra dijk;
        BFS bfs;
        AStar astar;

        for (const char* leaf : {"L1", "L2", "L3"}) {
            INFO("Hub -> " << leaf);
            CHECK(dijk.find_path(*g, "H", leaf).size() == 2);
            CHECK(bfs.find_path(*g, "H", leaf).size() == 2);
            CHECK(astar.find_path(*g, "H", leaf).size() == 2);
        }

        CHECK(dijk.find_path(*g, "L1", "L2").empty());
        CHECK(bfs.find_path(*g, "L1", "L2").empty());
        CHECK(astar.find_path(*g, "L1", "L2").empty());
    }

    TEST_CASE("Large linear chain – correctness at scale (200 nodes)") {
        const int N = 200;
        std::unique_ptr<TrafficGraph> g(new TrafficGraph());
        for (int i = 0; i < N; ++i) {
            std::string id = "N" + std::to_string(i);
            g->intersections[id] = std::make_unique<Intersection>(id, static_cast<double>(i), 0);
        }
        for (int i = 0; i + 1 < N; ++i) {
            std::string src = "N" + std::to_string(i);
            std::string dst = "N" + std::to_string(i + 1);
            std::string eid = src + "_" + dst;
            g->edges[eid] = std::make_unique<Road>(eid, g->intersections[src].get(), g->intersections[dst].get(), 1, 50, 12, true, 1.0);
        }

        Dijkstra dijk;
        BFS bfs;
        AStar astar;
        std::string last = "N" + std::to_string(N - 1);

        CHECK(static_cast<int>(dijk.find_path(*g, "N0", last).size()) == N);
        CHECK(static_cast<int>(bfs.find_path(*g, "N0", last).size()) == N);
        CHECK(static_cast<int>(astar.find_path(*g, "N0", last).size()) == N);
    }

    TEST_CASE("Runtime strategy swap – strategy pattern demonstration") {
        std::unique_ptr<TrafficGraph> g(new TrafficGraph());
        g->intersections["A"] = std::make_unique<Intersection>("A", 0, 0);
        g->intersections["B"] = std::make_unique<Intersection>("B", 10, 0);
        g->intersections["C"] = std::make_unique<Intersection>("C", 3, 3);
        g->intersections["D"] = std::make_unique<Intersection>("D", 7, 3);

        g->edges["AB"] = std::make_unique<Road>("AB", g->intersections["A"].get(), g->intersections["B"].get(), 1, 50, 12, true, 100.0);
        g->edges["AC"] = std::make_unique<Road>("AC", g->intersections["A"].get(), g->intersections["C"].get(), 1, 50, 12, true, 1.0);
        g->edges["CD"] = std::make_unique<Road>("CD", g->intersections["C"].get(), g->intersections["D"].get(), 1, 50, 12, true, 1.0);
        g->edges["DB"] = std::make_unique<Road>("DB", g->intersections["D"].get(), g->intersections["B"].get(), 1, 50, 12, true, 1.0);

        // Start with Dijkstra strategy
        std::unique_ptr<PathFinding> strategy(new Dijkstra());
        auto p1 = strategy->find_path(*g, "A", "B");
        CHECK(names_of(p1).size() == 4); // A→C→D→B (cheapest)

        // Swap to A* at runtime
        strategy.reset(new AStar());
        auto p2 = strategy->find_path(*g, "A", "B");
        CHECK(names_of(p2).size() == 4); // A→C→D→B (cheapest, same as Dijkstra)

        // Swap to BFS at runtime
        strategy.reset(new BFS());
        auto p3 = strategy->find_path(*g, "A", "B");
        CHECK(names_of(p3).size() == 2); // A→B (fewest hops)
    }

} // TEST_SUITE
