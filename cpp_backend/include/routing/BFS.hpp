#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include "routing/PathFinding.hpp"

class TrafficGraph;
class Intersection;
class Road;

class BFS final : public PathFinding {
public:
    std::vector<Intersection*> find_path(const TrafficGraph& graph,
                                 const std::string& start_id,
                                 const std::string& end_id) override;

private:
    std::queue<Intersection*> front;
    std::unordered_set<std::string> visited;
    std::unordered_map<std::string, Intersection*> prev;
    std::vector<Intersection*> reconstruct_path(const std::string& start_id,
                                         const std::string& end_id);
};
