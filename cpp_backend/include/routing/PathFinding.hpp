#pragma once
#include <cstddef>
#include <string>
#include <vector>

#include "core/Road.hpp"

class TrafficGraph;
class Intersection;

// Strategy interface for pathfinding algorithms
class PathFinding {
public:
    virtual ~PathFinding() = default;

    virtual std::vector<Intersection*> find_path(const TrafficGraph& graph,
                                         const std::string& start_id,
                                         const std::string& end_id) = 0;

    // What a route should minimise. Default Time: speed limits and live
    // congestion both matter, which is how the simulator prices roads.
    void set_cost_mode(CostMode mode) { cost_mode_ = mode; }
    CostMode cost_mode() const { return cost_mode_; }

    // How many nodes the last find_path() call settled. This is the search
    // effort the algorithm-comparison table reports: it is what separates A*
    // from Dijkstra, since both return the same route but A* should reach it
    // after expanding far fewer nodes.
    //
    // Deliberately NOT part of the return type: changing find_path()'s
    // signature would touch every call site across the test suite and buy
    // nothing else.
    std::size_t nodes_explored() const { return nodes_explored_; }

protected:
    CostMode cost_mode_ = CostMode::Time;
    // Each strategy resets this at the top of find_path() and increments it
    // once per node it settles (pops and commits to), never per edge.
    std::size_t nodes_explored_ = 0;
};
