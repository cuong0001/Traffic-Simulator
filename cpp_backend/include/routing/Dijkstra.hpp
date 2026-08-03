#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <queue>
#include "routing/PathFinding.hpp"
class TrafficGraph;
class Intersection;
class Road;

class Dijkstra final : public PathFinding {
public:
    std::vector<Intersection*> find_path(const TrafficGraph& graph,
                                 const std::string& start_id,
                                 const std::string& end_id) override;

private:
    std::unordered_map<std::string, double> dist;
    std::unordered_map<std::string, Intersection*> prev;
    std::unordered_map<std::string, Intersection*> node_of;
    using PQEntry = std::pair<double, std::string>;
    std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<PQEntry>> pq;
    void init_distances(const TrafficGraph& graph, const std::string& start_id);
    void relax_edge(Intersection* current, Road* road);
    std::vector<Intersection*> reconstruct_path(const std::string& start_id,
                                         const std::string& end_id);
};
