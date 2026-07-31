#pragma once
#include <cstddef>
#include <string>
#include <vector>

class TrafficGraph;
class Intersection;

// Strategy interface for pathfinding algorithms
class PathFinding {
public:
    virtual ~PathFinding() = default;

    virtual std::vector<Intersection*> find_path(const TrafficGraph& graph,
                                         const std::string& start_id,
                                         const std::string& end_id) = 0;
    std::size_t nodes_explored() const { return nodes_explored_; }

protected:
    std::size_t nodes_explored_ = 0;
};