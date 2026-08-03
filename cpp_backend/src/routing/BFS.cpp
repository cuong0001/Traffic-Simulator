#include "routing/BFS.hpp"
#include "core/Graph.hpp"
#include "core/Intersection.hpp"
#include "core/Road.hpp"

#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>

std::vector<Intersection*> BFS::find_path(const TrafficGraph& graph,
                                   const std::string& start_id,
                                   const std::string& end_id) {
    // clear state
    while (!front.empty()) front.pop();
    visited.clear();
    prev.clear();
    nodes_explored_ = 0;

    auto start_it = graph.intersections.find(start_id);
    auto end_it   = graph.intersections.find(end_id);
    if (start_it == graph.intersections.end() || end_it == graph.intersections.end())
        return {};

    Intersection* start_node = start_it->second.get();

    // Seed the BFS
    front.push(start_node);
    visited.insert(start_id);
    prev[start_id] = nullptr;          

    while (!front.empty()) {
        Intersection* current = front.front();
        front.pop();

        ++nodes_explored_;   // dequeued: this node is now expanded

        // Goal check
        if (current->name == end_id)
            return reconstruct_path(start_id, end_id);

        // Expand neighbours via outgoing roads
        for (Road* road : current->outgoing_edges) {
            if (road->is_closed()) continue;   // closed road is untraversable
            Intersection* neighbour = road->target;
            if (visited.find(neighbour->name) == visited.end()) {
                visited.insert(neighbour->name);
                prev[neighbour->name] = current;   // record parent
                front.push(neighbour);
            }
        }
    }

    // if there's no path found
    return {};
}

std::vector<Intersection*> BFS::reconstruct_path(const std::string& start_id,
                                          const std::string& end_id) {
    std::unordered_map<std::string, Intersection*> node_of;

    for (auto& [child_name, parent_ptr] : prev) {
        if (parent_ptr != nullptr) {
            // The child Intersection* lives in the parent's outgoing edges
            for (Road* road : parent_ptr->outgoing_edges) {
                if (road->target->name == child_name) {
                    node_of[child_name] = road->target;
                    break;
                }
            }
            // Also register the parent itself
            node_of[parent_ptr->name] = parent_ptr;
        }
    }

    // Walk backwards from end to start using the prev chain to trace the bfs path
    std::vector<Intersection*> path;
    std::string cur = end_id;

    while (true) {
        auto it = node_of.find(cur);
        if (it == node_of.end())
            return {};                  // should never happen on a valid call
        path.push_back(it->second);

        if (cur == start_id)
            break;

        cur = prev[cur]->name;
    }

    std::reverse(path.begin(), path.end());
    return path;
}
