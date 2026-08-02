#include "routing/Dijkstra.hpp"
#include "core/Graph.hpp"
#include "core/Intersection.hpp"
#include "core/Road.hpp"

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
    node_of.clear();
    while (!pq.empty()) pq.pop();
    nodes_explored_ = 0;

    // validate that both nodes exist in the graph
    if (graph.intersections.find(start_id) == graph.intersections.end() ||
        graph.intersections.find(end_id)   == graph.intersections.end())
        return {};

    // Check if the start and end are the same node
    if (start_id == end_id)
        return { graph.intersections.at(start_id) };

    init_distances(graph, start_id);

    while (!pq.empty()) {
        auto [cost, current_name] = pq.top();
        pq.pop();

        // skip relaxed nodes
        if (cost > dist[current_name])
            continue;

        ++nodes_explored_;   // settled: this node's distance is now final

        // Exit early if we can find a shortest path
        if (current_name == end_id)
            return reconstruct_path(start_id, end_id);

        Intersection* current = graph.intersections.at(current_name);

        // Relax all outgoing edges
        for (Road* road : current->outgoing_edges) {
            if (road->is_closed) continue;   // closed road is untraversable
            relax_edge(current, road);
        }
    }

    // No path exists
    return {};
}

void Dijkstra::init_distances(const TrafficGraph& graph,
                               const std::string& start_id) {
    // Lazy init: "absent from dist" already means infinity, so there is no
    // reason to write 2617 infinities before a search that settles a few
    // hundred nodes.
    Intersection* start = graph.intersections.at(start_id);
    dist[start_id] = 0.0;
    prev[start_id] = nullptr;
    node_of[start_id] = start;
    pq.push({ 0.0, start_id });
}

void Dijkstra::relax_edge(Intersection* current, Road* road) {
    Intersection* neighbour = road->target;
    double new_dist = dist[current->name] + road->get_weight();

    // find() rather than operator[]: an id we have never reached is absent,
    // which is exactly "distance infinity".
    auto it = dist.find(neighbour->name);
    if (it == dist.end() || new_dist < it->second) {
        dist[neighbour->name] = new_dist;
        prev[neighbour->name] = current;
        node_of[neighbour->name] = neighbour;
        pq.push({ new_dist, neighbour->name });
    }
}

std::vector<Intersection*> Dijkstra::reconstruct_path(const std::string& start_id,
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
