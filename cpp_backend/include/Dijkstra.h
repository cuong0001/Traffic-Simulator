#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <queue>
using namespace std;
class TrafficGraph;
class Node;
class Road;

class Dijkstra {
public:
    std::vector<Node*> find_path(const TrafficGraph& graph,
                                 const std::string& start_id,
                                 const std::string& end_id);

private:
    std::unordered_map<std::string, double> dist;
    std::unordered_map<std::string, Node*> prev;
    using PQEntry = std::pair<double, std::string>;
    std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<PQEntry>> pq;
    void init_distances(const TrafficGraph& graph, const std::string& start_id);
    void relax_edge(Node* current, Road* road);
    std::vector<Node*> reconstruct_path(const std::string& start_id,
                                         const std::string& end_id);
};
