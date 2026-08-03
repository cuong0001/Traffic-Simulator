#include "routing/AStar.hpp"
#include "core/Graph.hpp"
#include "core/Intersection.hpp"
#include "core/Road.hpp"

#include <queue>
#include <unordered_map>
#include <algorithm>
#include <limits>
#include <cmath>

// Euclidean (straight-line) distance heuristic.
//
// Admissibility: h(a, b) must never overestimate the true cost from a to the
// goal, which is what guarantees A* returns an optimal path. This holds when
// every edge's traversal cost (travel_cost) is at least the Euclidean distance
// between its endpoints -- the normal case for real roads, since a road cannot
// be shorter than the straight line joining its ends and travel_cost >=
// base_weight = length. By the triangle inequality, the straight-line distance
// from any node to the goal is then a lower bound on the remaining path cost.
//
// Skipping closed roads only removes edges from the graph, which can only
// raise the true shortest-path cost; the heuristic is unchanged and therefore
// remains admissible.
double AStar::heuristic(Intersection* a, Intersection* b) {
    double dx = a->x - b->x;
    double dy = a->y - b->y;
    double h = std::sqrt(dx * dx + dy * dy);
    // For Time mode, the admissible bound is "shortest possible time":
    // straight-line distance over the fastest speed any road allows. Without
    // this, h can overestimate and A* stops being optimal. Distance mode
    // keeps the plain Euclidean distance.
    if (cost_mode_ == CostMode::Time && max_speed_ms_ > 0.0)
        h /= max_speed_ms_;
    return h;
}

std::vector<Intersection*> AStar::find_path(const TrafficGraph& graph,
                                        const std::string& start_id,
                                        const std::string& end_id) {
    // clear state
    g_score.clear();
    f_score.clear();
    prev.clear();
    node_of.clear();
    while (!pq.empty()) pq.pop();
    nodes_explored_ = 0;

    // validate that both nodes exist in the graph
    if (graph.intersections.find(start_id) == graph.intersections.end() ||
        graph.intersections.find(end_id)   == graph.intersections.end())
        return {};

    if (start_id == end_id)
        return { graph.intersections.at(start_id).get() };

    // In Time mode the admissible heuristic is distance / fastest speed on
    // the map. Recompute the max speed once per query so it stays fresh after
    // live map edits; a tight no-allocation loop is negligible next to the
    // search itself.
    if (cost_mode_ == CostMode::Time) {
        max_speed_ms_ = 0.0;
        for (const auto& [id, road] : graph.edges)
            if (road) max_speed_ms_ = std::max(max_speed_ms_, road->speed_limit_ms());
    }

    init_scores(graph, start_id, end_id);
    Intersection* goal = graph.intersections.at(end_id).get();

    while (!pq.empty()) {
        auto [f, current_name] = pq.top();
        pq.pop();

        // skip stale entries
        if (f > f_score[current_name])
            continue;

        ++nodes_explored_;   // settled: this node's distance is now final

        // exit early when goal is reached
        if (current_name == end_id)
            return reconstruct_path(start_id, end_id);

        Intersection* current = graph.intersections.at(current_name).get();

        for (Road* road : current->outgoing_edges) {
            if (road->is_closed()) continue;   // closed road is untraversable
            Intersection* neighbour = road->target;
            double tentative_g = g_score[current_name] + road->cost(cost_mode_);

            // find() rather than operator[]: an id we have never reached is
            // absent from g_score, which is exactly "cost so far infinity".
            auto it = g_score.find(neighbour->name);
            if (it == g_score.end() || tentative_g < it->second) {
                prev[neighbour->name] = current;
                node_of[neighbour->name] = neighbour;
                g_score[neighbour->name] = tentative_g;
                f_score[neighbour->name] = tentative_g + heuristic(neighbour, goal);
                pq.push({ f_score[neighbour->name], neighbour->name });
            }
        }
    }

    return {};
}

void AStar::init_scores(const TrafficGraph& graph,
                        const std::string& start_id,
                        const std::string& end_id) {
    // Lazy init: "absent from g_score" already means infinity, so there is no
    // reason to write 2617 infinities before a search that settles a few
    // hundred nodes.
    Intersection* goal = graph.intersections.at(end_id).get();
    Intersection* start = graph.intersections.at(start_id).get();

    g_score[start_id] = 0.0;
    f_score[start_id] = heuristic(start, goal);
    prev[start_id] = nullptr;
    node_of[start_id] = start;
    pq.push({ f_score[start_id], start_id });
}

std::vector<Intersection*> AStar::reconstruct_path(const std::string& start_id,
                                               const std::string& end_id) {
    // node_of was filled during relaxation, so the path is just a walk back.
    // The old version scanned the whole prev map and every parent's outgoing
    // edges to rebuild information it was already holding.
    std::vector<Intersection*> path;
    std::string cur = end_id;

    while (true) {
        auto it = node_of.find(cur);
        if (it == node_of.end())
            return {};
        path.push_back(it->second);

        if (cur == start_id)
            break;

        auto pit = prev.find(cur);
        if (pit == prev.end() || pit->second == nullptr)
            return {};
        cur = pit->second->name;
    }

    std::reverse(path.begin(), path.end());
    return path;
}
