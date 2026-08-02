// General unit tests for Dijkstra, BFS & A* through the PathFinding strategy interface.
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
//   cmake --build build --target test_pathfinder_unit
//   ./build/Debug/test_pathfinder_unit            (Windows)
//   ./build/test_pathfinder_unit                  (Linux/Mac)
//
// Adding a new test:
//   1. Write a TEST_CASE inside the existing TEST_SUITE block.
//   2. Build a TrafficGraph in-memory (see helpers below) or write your own.
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

// Collect the intersection names along a path for easy comparison.
static std::vector<std::string> names_of(const std::vector<Intersection*>& path) {
    std::vector<std::string> out;
    out.reserve(path.size());
    for (Intersection* n : path) out.push_back(n->name);
    return out;
}


//      A --10--> B --10--> D
//      |                   ^
//      +---5---> C ---5----+
//
// The cheap path A→D is A→C→D (cost 10), not A→B→D (cost 20).
static TrafficGraph* make_diamond_graph() {
    TrafficGraph* g = new TrafficGraph();

    g->intersections["A"] = std::make_unique<Intersection>("A", 0, 0);
    g->intersections["B"] = std::make_unique<Intersection>("B", 10, 0);
    g->intersections["C"] = std::make_unique<Intersection>("C", 5, 5);
    g->intersections["D"] = std::make_unique<Intersection>("D", 20, 0);

    g->edges["AB"] = std::make_unique<Road>("AB", g->intersections["A"].get(), g->intersections["B"].get(), 1, 50, 12, true, 10.0);
    g->edges["BD"] = std::make_unique<Road>("BD", g->intersections["B"].get(), g->intersections["D"].get(), 1, 50, 12, true, 10.0);
    g->edges["AC"] = std::make_unique<Road>("AC", g->intersections["A"].get(), g->intersections["C"].get(), 1, 50, 12, true, 5.0);
    g->edges["CD"] = std::make_unique<Road>("CD", g->intersections["C"].get(), g->intersections["D"].get(), 1, 50, 12, true, 5.0);

    return g;
}

// Build a linear chain:  N0 → N1 → N2 → ... → N(count-1), each edge length 1.
static TrafficGraph* make_chain_graph(int count) {
    TrafficGraph* g = new TrafficGraph();
    for (int i = 0; i < count; ++i) {
        std::string id = "N" + std::to_string(i);
        g->intersections[id] = std::make_unique<Intersection>(id, static_cast<double>(i), 0);
    }
    for (int i = 0; i + 1 < count; ++i) {
        std::string src = "N" + std::to_string(i);
        std::string dst = "N" + std::to_string(i + 1);
        std::string eid = src + "_" + dst;
        g->edges[eid] = std::make_unique<Road>(eid, g->intersections[src].get(), g->intersections[dst].get(), 1, 50, 12, true, 1.0);
    }
    return g;
}

TEST_SUITE("PathFinding – Unit Tests") {

    TEST_CASE("Strategy polymorphism – all algorithms work through PathFinding*") {
        std::unique_ptr<TrafficGraph> g(make_diamond_graph());

        std::unique_ptr<PathFinding> dijk(new Dijkstra());
        std::unique_ptr<PathFinding> bfs(new BFS());
        std::unique_ptr<PathFinding> astar(new AStar());

        auto path_d = dijk->find_path(*g, "A", "D");
        auto path_b = bfs->find_path(*g, "A", "D");
        auto path_a = astar->find_path(*g, "A", "D");

        REQUIRE_FALSE(path_d.empty());
        REQUIRE_FALSE(path_b.empty());
        REQUIRE_FALSE(path_a.empty());

        CHECK(path_d.front()->name == "A");
        CHECK(path_d.back()->name  == "D");
        CHECK(path_b.front()->name == "A");
        CHECK(path_b.back()->name  == "D");
        CHECK(path_a.front()->name == "A");
        CHECK(path_a.back()->name  == "D");
    }

    TEST_CASE("Dijkstra picks the lowest-cost path") {
        std::unique_ptr<TrafficGraph> g(make_diamond_graph());
        Dijkstra dijk;

        auto path = dijk.find_path(*g, "A", "D");
        auto ids  = names_of(path);

        // Shortest-weight path: A→C→D (cost 10), not A→B→D (cost 20)
        REQUIRE(ids.size() == 3);
        CHECK(ids[0] == "A");
        CHECK(ids[1] == "C");
        CHECK(ids[2] == "D");
    }

    TEST_CASE("A* finds a valid weighted path") {
        std::unique_ptr<TrafficGraph> g(make_diamond_graph());
        AStar astar;

        auto path = astar.find_path(*g, "A", "D");
        auto ids  = names_of(path);

        // A* finds a path; exact route depends on heuristic vs. edge weights
        REQUIRE(ids.size() == 3);
        CHECK(ids.front() == "A");
        CHECK(ids.back()  == "D");
    }

    TEST_CASE("BFS finds a valid path") {
        std::unique_ptr<TrafficGraph> g(make_diamond_graph());
        BFS bfs;

        auto path = bfs.find_path(*g, "A", "D");
        auto ids  = names_of(path);

        // Both routes have 2 hops (A→B→D or A→C→D), BFS may return either
        REQUIRE(ids.size() == 3);
        CHECK(ids.front() == "A");
        CHECK(ids.back()  == "D");
    }

    TEST_CASE("Same start and end returns a single-node path") {
        std::unique_ptr<TrafficGraph> g(make_diamond_graph());

        Dijkstra dijk;
        auto path_d = dijk.find_path(*g, "A", "A");
        REQUIRE(path_d.size() == 1);
        CHECK(path_d[0]->name == "A");

        // BFS goal-check on dequeue may yield size 1 or empty
        BFS bfs;
        auto path_b = bfs.find_path(*g, "A", "A");
        CHECK((path_b.size() == 1 || path_b.empty()));

        AStar astar;
        auto path_a = astar.find_path(*g, "A", "A");
        REQUIRE(path_a.size() == 1);
        CHECK(path_a[0]->name == "A");
    }

    TEST_CASE("Linear chain – only one possible path") {
        std::unique_ptr<TrafficGraph> g(make_chain_graph(5)); // N0→N1→N2→N3→N4

        Dijkstra dijk;
        auto pd = dijk.find_path(*g, "N0", "N4");
        REQUIRE(pd.size() == 5);
        CHECK(pd.front()->name == "N0");
        CHECK(pd.back()->name  == "N4");

        BFS bfs;
        auto pb = bfs.find_path(*g, "N0", "N4");
        REQUIRE(pb.size() == 5);
        CHECK(pb.front()->name == "N0");
        CHECK(pb.back()->name  == "N4");

        AStar astar;
        auto pa = astar.find_path(*g, "N0", "N4");
        REQUIRE(pa.size() == 5);
        CHECK(pa.front()->name == "N0");
        CHECK(pa.back()->name  == "N4");
    }

    TEST_CASE("Dijkstra prefers a cheaper longer route over an expensive direct edge") {
        //   A --(100)--> B
        //   A -(1)-> C -(1)-> D -(1)-> B   (detour cost = 3)
        std::unique_ptr<TrafficGraph> g(new TrafficGraph());
        g->intersections["A"] = std::make_unique<Intersection>("A", 0, 0);
        g->intersections["B"] = std::make_unique<Intersection>("B", 10, 0);
        g->intersections["C"] = std::make_unique<Intersection>("C", 3, 3);
        g->intersections["D"] = std::make_unique<Intersection>("D", 7, 3);

        g->edges["AB"] = std::make_unique<Road>("AB", g->intersections["A"].get(), g->intersections["B"].get(), 1, 50, 12, true, 100.0);
        g->edges["AC"] = std::make_unique<Road>("AC", g->intersections["A"].get(), g->intersections["C"].get(), 1, 50, 12, true, 1.0);
        g->edges["CD"] = std::make_unique<Road>("CD", g->intersections["C"].get(), g->intersections["D"].get(), 1, 50, 12, true, 1.0);
        g->edges["DB"] = std::make_unique<Road>("DB", g->intersections["D"].get(), g->intersections["B"].get(), 1, 50, 12, true, 1.0);

        Dijkstra dijk;
        auto ids = names_of(dijk.find_path(*g, "A", "B"));

        REQUIRE(ids.size() == 4); // A→C→D→B
        CHECK(ids[0] == "A");
        CHECK(ids[1] == "C");
        CHECK(ids[2] == "D");
        CHECK(ids[3] == "B");
    }

    TEST_CASE("A* prefers a cheaper longer route over an expensive direct edge") {
        std::unique_ptr<TrafficGraph> g(new TrafficGraph());
        g->intersections["A"] = std::make_unique<Intersection>("A", 0, 0);
        g->intersections["B"] = std::make_unique<Intersection>("B", 10, 0);
        g->intersections["C"] = std::make_unique<Intersection>("C", 3, 3);
        g->intersections["D"] = std::make_unique<Intersection>("D", 7, 3);

        g->edges["AB"] = std::make_unique<Road>("AB", g->intersections["A"].get(), g->intersections["B"].get(), 1, 50, 12, true, 100.0);
        g->edges["AC"] = std::make_unique<Road>("AC", g->intersections["A"].get(), g->intersections["C"].get(), 1, 50, 12, true, 1.0);
        g->edges["CD"] = std::make_unique<Road>("CD", g->intersections["C"].get(), g->intersections["D"].get(), 1, 50, 12, true, 1.0);
        g->edges["DB"] = std::make_unique<Road>("DB", g->intersections["D"].get(), g->intersections["B"].get(), 1, 50, 12, true, 1.0);

        AStar astar;
        auto ids = names_of(astar.find_path(*g, "A", "B"));

        REQUIRE(ids.size() == 4); // A→C→D→B
        CHECK(ids[0] == "A");
        CHECK(ids[1] == "C");
        CHECK(ids[2] == "D");
        CHECK(ids[3] == "B");
    }

    TEST_CASE("BFS returns the fewest-hop path regardless of weight") {
        // Same graph: BFS should prefer the 1-hop A→B over the 3-hop detour
        std::unique_ptr<TrafficGraph> g(new TrafficGraph());
        g->intersections["A"] = std::make_unique<Intersection>("A", 0, 0);
        g->intersections["B"] = std::make_unique<Intersection>("B", 10, 0);
        g->intersections["C"] = std::make_unique<Intersection>("C", 3, 3);
        g->intersections["D"] = std::make_unique<Intersection>("D", 7, 3);

        g->edges["AB"] = std::make_unique<Road>("AB", g->intersections["A"].get(), g->intersections["B"].get(), 1, 50, 12, true, 100.0);
        g->edges["AC"] = std::make_unique<Road>("AC", g->intersections["A"].get(), g->intersections["C"].get(), 1, 50, 12, true, 1.0);
        g->edges["CD"] = std::make_unique<Road>("CD", g->intersections["C"].get(), g->intersections["D"].get(), 1, 50, 12, true, 1.0);
        g->edges["DB"] = std::make_unique<Road>("DB", g->intersections["D"].get(), g->intersections["B"].get(), 1, 50, 12, true, 1.0);

        BFS bfs;
        auto ids = names_of(bfs.find_path(*g, "A", "B"));

        REQUIRE(ids.size() == 2); // A→B
        CHECK(ids[0] == "A");
        CHECK(ids[1] == "B");
    }

    TEST_CASE("Algorithm object can be reused across multiple queries") {
        std::unique_ptr<TrafficGraph> g(make_diamond_graph());

        Dijkstra dijk;
        auto p1 = dijk.find_path(*g, "A", "D");
        auto p2 = dijk.find_path(*g, "A", "B");
        auto p3 = dijk.find_path(*g, "A", "D");

        REQUIRE_FALSE(p1.empty());
        REQUIRE_FALSE(p2.empty());
        CHECK(names_of(p1) == names_of(p3));

        BFS bfs;
        auto b1 = bfs.find_path(*g, "A", "D");
        auto b2 = bfs.find_path(*g, "A", "B");
        auto b3 = bfs.find_path(*g, "A", "D");

        REQUIRE_FALSE(b1.empty());
        REQUIRE_FALSE(b2.empty());
        CHECK(names_of(b1) == names_of(b3));

        AStar astar;
        auto a1 = astar.find_path(*g, "A", "D");
        auto a2 = astar.find_path(*g, "A", "B");
        auto a3 = astar.find_path(*g, "A", "D");

        REQUIRE_FALSE(a1.empty());
        REQUIRE_FALSE(a2.empty());
        CHECK(names_of(a1) == names_of(a3));
    }

    TEST_CASE("Direct neighbour – single edge path") {
        std::unique_ptr<TrafficGraph> g(make_diamond_graph());

        Dijkstra dijk;
        auto pd = dijk.find_path(*g, "A", "B");
        REQUIRE(pd.size() == 2);
        CHECK(pd[0]->name == "A");
        CHECK(pd[1]->name == "B");

        BFS bfs;
        auto pb = bfs.find_path(*g, "A", "B");
        REQUIRE(pb.size() == 2);
        CHECK(pb[0]->name == "A");
        CHECK(pb[1]->name == "B");

        AStar astar;
        auto pa = astar.find_path(*g, "A", "B");
        REQUIRE(pa.size() == 2);
        CHECK(pa[0]->name == "A");
        CHECK(pa[1]->name == "B");
    }

    TEST_CASE("All algorithms agree on reachability") {
        std::unique_ptr<TrafficGraph> g(make_diamond_graph());

        Dijkstra dijk;
        BFS bfs;
        AStar astar;

        // Reachable pairs
        std::vector<std::pair<std::string,std::string>> reachable = {
            {"A","B"}, {"A","C"}, {"A","D"}, {"B","D"}, {"C","D"}
        };
        for (auto& [s, e] : reachable) {
            INFO("Reachable: " << s << " -> " << e);
            CHECK_FALSE(dijk.find_path(*g, s, e).empty());
            CHECK_FALSE(bfs.find_path(*g, s, e).empty());
            CHECK_FALSE(astar.find_path(*g, s, e).empty());
        }

        // Unreachable (reversed direction in a DAG)
        std::vector<std::pair<std::string,std::string>> unreachable = {
            {"D","A"}, {"B","A"}, {"D","C"}
        };
        for (auto& [s, e] : unreachable) {
            INFO("Unreachable: " << s << " -> " << e);
            CHECK(dijk.find_path(*g, s, e).empty());
            CHECK(bfs.find_path(*g, s, e).empty());
            CHECK(astar.find_path(*g, s, e).empty());
        }
    }

    TEST_CASE("Closed road forces a detour in all three strategies") {
        // Diamond graph: A->C->D (cost 10) is the cheap route, A->B->D
        // (cost 20) the expensive one. Closing A->C must make every
        // algorithm use A->B->D regardless of weights or heuristic.
        std::unique_ptr<TrafficGraph> g(make_diamond_graph());
        g->edges["AC"]->is_closed = true;

        Dijkstra dijk;
        BFS bfs;
        AStar astar;
        std::vector<std::pair<std::string, PathFinding*>> algos = {
            {"Dijkstra", &dijk}, {"BFS", &bfs}, {"AStar", &astar}
        };

        for (auto& [name, algo] : algos) {
            INFO("Algorithm: " << name);
            auto ids = names_of(algo->find_path(*g, "A", "D"));

            REQUIRE(ids.size() == 3);
            CHECK(ids[0] == "A");
            CHECK(ids[1] == "B");
            CHECK(ids[2] == "D");
        }
    }

    TEST_CASE("Closing the only route makes the destination unreachable") {
        // Linear chain N0->N1->N2->N3->N4. Closing N2->N3 cuts off N3 and N4,
        // so there is no path from N0 to N4.
        std::unique_ptr<TrafficGraph> g(make_chain_graph(5));
        g->edges["N2_N3"]->is_closed = true;

        Dijkstra dijk;
        BFS bfs;
        AStar astar;
        std::vector<std::pair<std::string, PathFinding*>> algos = {
            {"Dijkstra", &dijk}, {"BFS", &bfs}, {"AStar", &astar}
        };

        for (auto& [name, algo] : algos) {
            INFO("Algorithm: " << name);
            CHECK(algo->find_path(*g, "N0", "N4").empty());
        }
    }

} // TEST_SUITE
