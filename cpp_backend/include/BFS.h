#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
using namespace std;

class TrafficGraph;
class Node;
class Road;

class BFS {
public:
    std::vector<Node*> find_path(const TrafficGraph& graph,
                                 const std::string& start_id,
                                 const std::string& end_id);

private:
    std::queue<Node*> front;
    std::unordered_set<std::string> visited;
    std::unordered_map<std::string, Node*> prev;
    std::vector<Node*> reconstruct_path(const std::string& start_id,
                                         const std::string& end_id);
};
