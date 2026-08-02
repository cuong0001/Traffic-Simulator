#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <queue>
#include "routing/PathFinding.hpp"
using namespace std;

class TrafficGraph;
class Intersection;
class Road;

class AStar : public PathFinding {
public:
    std::vector<Intersection*> find_path(const TrafficGraph& graph,
                                 const std::string& start_id,
                                 const std::string& end_id) override;

private:
    std::unordered_map<std::string, double> g_score;
    std::unordered_map<std::string, double> f_score;
    std::unordered_map<std::string, Intersection*> prev;
    std::unordered_map<std::string, Intersection*> node_of;
    using PQEntry = std::pair<double, std::string>;
    std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<PQEntry>> pq;

    void init_scores(const TrafficGraph& graph, const std::string& start_id,
                     const std::string& end_id);
    double heuristic(Intersection* a, Intersection* b);
    double max_speed_ms_ = 0.0;   // fastest speed limit on the map, set per query in Time mode
    std::vector<Intersection*> reconstruct_path(const std::string& start_id,
                                         const std::string& end_id);
};
