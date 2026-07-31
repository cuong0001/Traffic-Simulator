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
    return std::sqrt(dx * dx + dy * dy);
}

std::vector<Intersection*> AStar::find_path(const TrafficGraph& graph,
                                        const std::string& start_id,
                                        const std::string& end_id) {
    // clear state
    g_score.clear();
    f_score.clear();
    prev.clear();
    while (!pq.empty()) pq.pop();
    nodes_explored_ = 0;

    // validate that both nodes exist in the graph
    if (graph.intersections.find(start_id) == graph.intersections.end() ||
        graph.intersections.find(end_id)   == graph.intersections.end())
        return {};

    if (start_id == end_id)
        return { graph.intersections.at(start_id) };

    init_scores(graph, start_id, end_id);

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

        Intersection* current = graph.intersections.at(current_name);

        for (Road* road : current->outgoing_edges) {
            if (road->is_closed) continue;   // closed road is untraversable
            Intersection* neighbour = road->target;
            double tentative_g = g_score[current_name] + road->get_weight();

            if (tentative_g < g_score[neighbour->name]) {
                prev[neighbour->name] = current;
                g_score[neighbour->name] = tentative_g;
                f_score[neighbour->name] = tentative_g +
                    heuristic(neighbour, graph.intersections.at(end_id));
                pq.push({ f_score[neighbour->name], neighbour->name });
            }
        }
    }

    return {};
}

void AStar::init_scores(const TrafficGraph& graph,
                        const std::string& start_id,
                        const std::string& end_id) {
    Intersection* goal = graph.intersections.at(end_id);

    for (auto& [name, node] : graph.intersections) {
        g_score[name] = std::numeric_limits<double>::infinity();
        f_score[name] = std::numeric_limits<double>::infinity();
        prev[name] = nullptr;
    }

    g_score[start_id] = 0.0;
    f_score[start_id] = heuristic(graph.intersections.at(start_id), goal);
    pq.push({ f_score[start_id], start_id });
}

std::vector<Intersection*> AStar::reconstruct_path(const std::string& start_id,
                                               const std::string& end_id) {
    std::unordered_map<std::string, Intersection*> node_of;

    for (auto& [child_name, parent_ptr] : prev) {
        if (parent_ptr != nullptr) {
            node_of[parent_ptr->name] = parent_ptr;

            for (Road* road : parent_ptr->outgoing_edges) {
                if (road->target->name == child_name) {
                    node_of[child_name] = road->target;
                    break;
                }
            }
        }
    }

    std::vector<Intersection*> path;
    std::string cur = end_id;

    while (true) {
        auto it = node_of.find(cur);
        if (it == node_of.end())
            return {};
        path.push_back(it->second);

        if (cur == start_id)
            break;

        cur = prev[cur]->name;
    }

    std::reverse(path.begin(), path.end());
    return path;
}
