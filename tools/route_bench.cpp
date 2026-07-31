// Runs BFS, Dijkstra and A* over the same OD pairs on a real map and reports,
// per algorithm: nodes settled, time, route cost and road count. Exits 0 only
// if A* and Dijkstra agree on cost for every pair they both solved.
//
//   Build: cmake --build build_release --target route_bench
//   Run: build_release\route_bench.exe data\maps\small_map.geojson 100

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "core/Graph.hpp"
#include "core/Intersection.hpp"
#include "core/Road.hpp"
#include "routing/AStar.hpp"
#include "routing/BFS.hpp"
#include "routing/Dijkstra.hpp"
#include "routing/PathFinding.hpp"
#include "utils/MapLoader.hpp"

using namespace std;

struct Tally {
    string name;
    int solved = 0;
    double nodes = 0.0;
    double micros = 0.0;
    double cost = 0.0;
    double hops = 0.0;
};

// Sum of road weights along a node path; -1 if two consecutive nodes are not adjacent.
static double path_cost(const vector<Intersection*>& path) {
    double total = 0.0;
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        const Road* best = nullptr;
        for (const Road* r : path[i]->outgoing_edges) {
            if (r->target != path[i + 1]) continue;
            if (!best || r->get_weight() < best->get_weight()) best = r;
        }
        if (!best) return -1.0;
        total += best->get_weight();
    }
    return total;
}

int main(int argc, char** argv) {
    const string map_path = argc > 1 ? argv[1] : "data/maps/small_map.geojson";
    const int wanted = argc > 2 ? atoi(argv[2]) : 40;

    TrafficGraph graph;
    try {
        MapLoader::load_from_geojson(graph, map_path);
    } catch (const MapLoadError& e) {
        fprintf(stderr, "cannot load %s: %s\n", map_path.c_str(), e.what());
        return 1;
    }

    vector<string> ids;
    ids.reserve(graph.intersections.size());
    for (const auto& [id, node] : graph.intersections) ids.push_back(id);
    if (ids.size() < 2) {
        fprintf(stderr, "map has too few nodes\n");
        return 1;
    }
    sort(ids.begin(), ids.end());

    Dijkstra dijkstra;
    AStar astar;
    BFS bfs;
    PathFinding* strategies[] = {&bfs, &dijkstra, &astar};
    Tally tally[] = {{"BFS"}, {"Dijkstra"}, {"A*"}};

    int attempted = 0, comparable = 0, cost_match = 0;

    for (int i = 0; i < wanted; ++i) {
        const string from = ids[(static_cast<size_t>(i) * 131) % ids.size()];
        const string to   = ids[(static_cast<size_t>(i) * 71 + 9) % ids.size()];
        if (from == to) continue;
        ++attempted;

        double cost_of[3]   = {-1.0, -1.0, -1.0};
        double hops_of[3]   = {0.0, 0.0, 0.0};
        double nodes_of[3]  = {0.0, 0.0, 0.0};
        double us_of[3]     = {0.0, 0.0, 0.0};
        bool   found_by[3]  = {false, false, false};

        for (int s = 0; s < 3; ++s) {
            const auto t0 = chrono::steady_clock::now();
            const vector<Intersection*> path =
                strategies[s]->find_path(graph, from, to);
            const double us = chrono::duration<double, micro>(
                                  chrono::steady_clock::now() - t0).count();

            us_of[s]    = us;
            nodes_of[s] = static_cast<double>(strategies[s]->nodes_explored());

            if (path.empty()) continue;
            found_by[s] = true;
            ++tally[s].solved;
            cost_of[s] = path_cost(path);
            hops_of[s] = static_cast<double>(path.size() - 1);
        }

        if (!(found_by[0] && found_by[1] && found_by[2])) continue;
        ++comparable;
        for (int s = 0; s < 3; ++s) {
            tally[s].nodes  += nodes_of[s];
            tally[s].micros += us_of[s];
            tally[s].cost   += cost_of[s];
            tally[s].hops   += hops_of[s];
        }
        if (fabs(cost_of[1] - cost_of[2]) < 1e-6) ++cost_match;
    }

    printf("map           : %s\n", map_path.c_str());
    printf("graph         : %zu intersections, %zu roads\n",
            graph.intersections.size(), graph.edges.size());
    printf("pairs         : %d attempted, %d solved by all three\n\n",
            attempted, comparable);

    printf("%-10s %8s %12s %12s %14s %10s\n",
            "Algorithm", "Solved", "Nodes/route", "Time (us)",
            "Route cost", "Roads");
    printf("%-10s %8s %12s %12s %14s %10s\n",
            "----------", "------", "-----------", "---------",
            "----------", "-----");
    for (int s = 0; s < 3; ++s) {
        const Tally& t = tally[s];
        const double per = comparable ? t.nodes / comparable : 0.0;
        const double us  = comparable ? t.micros / comparable : 0.0;
        printf("%-10s %8d %12.1f %12.1f %14.1f %10.1f\n",
                t.name.c_str(), t.solved, per, us,
                comparable ? t.cost / comparable : 0.0,
                comparable ? t.hops / comparable : 0.0);
    }

    const double dij_nodes = comparable ? tally[1].nodes : 0.0;
    const double ast_nodes = comparable ? tally[2].nodes : 0.0;

    printf("\n--- What the table shows ---\n");
    printf("A* == Dijkstra cost : %d / %d pairs%s\n", cost_match, comparable,
            cost_match == comparable ? "   (admissible heuristic => optimal)"
                                     : "   *** MISMATCH ***");
    printf("A* search effort    : %.1f%% of Dijkstra's nodes\n",
            dij_nodes > 0 ? 100.0 * ast_nodes / dij_nodes : 0.0);
    printf("BFS route cost      : %.1f%% of Dijkstra's%s\n",
            tally[1].cost > 0 ? 100.0 * tally[0].cost / tally[1].cost : 0.0,
            tally[0].cost > tally[1].cost + 1e-6
                ? "   (more expensive: BFS counts roads, not weight)"
                : "");

    // A cost mismatch means a broken heuristic which is a fail.
    return (comparable > 0 && cost_match == comparable) ? 0 : 1;
}
