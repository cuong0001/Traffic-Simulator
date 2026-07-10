#pragma once
#include <string>
#include <vector>

class TrafficGraph;
class Node;

// Strategy interface for pathfinding algorithms
class PathFinding {
public:
    virtual ~PathFinding() = default;

    virtual std::vector<Node*> find_path(const TrafficGraph& graph,
                                         const std::string& start_id,
                                         const std::string& end_id) = 0;
};
