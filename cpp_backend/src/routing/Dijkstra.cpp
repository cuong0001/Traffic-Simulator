#include "../../include/Dijkstra.h"
#include "../../include/Graph.h"
#include "../../include/Intersection.h"
#include "../../include/Road.h"

#include <queue>
#include <unordered_map>
#include <algorithm>
#include <limits>

std::vector<Intersection*> Dijkstra::find_path(const TrafficGraph& graph,
                                        const std::string& start_id,
                                        const std::string& end_id) {
    // clear state 
    dist.clear();
    prev.clear();
    while (!pq.empty()) pq.pop();

    // validate that both nodes exist in the graph
    if (graph.nodes.find(start_id) == graph.nodes.end() ||
        graph.nodes.find(end_id)   == graph.nodes.end())
        return {};

    // Check if the start and end are the same node
    if (start_id == end_id)
        return { graph.nodes.at(start_id) };

    init_distances(graph, start_id);

    while (!pq.empty()) {
        auto [cost, current_name] = pq.top();
        pq.pop();

        // skip relaxed nodes
        if (cost > dist[current_name])
            continue;

        // Exit early if we can find a shortest path
        if (current_name == end_id)
            return reconstruct_path(start_id, end_id);

        Intersection* current = graph.nodes.at(current_name);

        // Relax all outgoing edges
        for (Road* road : current->outgoing_edges) {
            relax_edge(current, road);
        }
    }

    // No path exists
    return {};
}

void Dijkstra::init_distances(const TrafficGraph& graph,
                               const std::string& start_id) {
    // Set all distances to infinity
    for (auto& [name, node] : graph.nodes) {
        dist[name] = std::numeric_limits<double>::infinity();
        prev[name] = nullptr;
    }

    // Start node has distance 0
    dist[start_id] = 0.0;
    pq.push({ 0.0, start_id });
}

void Dijkstra::relax_edge(Intersection* current, Road* road) {
    Intersection* neighbour = road->target;
    double new_dist = dist[current->name] + road->get_weight();

    if (new_dist < dist[neighbour->name]) {
        dist[neighbour->name] = new_dist;
        prev[neighbour->name] = current;
        pq.push({ new_dist, neighbour->name });
    }
}

std::vector<Intersection*> Dijkstra::reconstruct_path(const std::string& start_id,
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

    // Walk backwards from end to start following the prev chain to trace the dijkstra path
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
