#ifndef GRAPH_H
#define GRAPH_H

#include <string>
#include <map>
#include <vector>
#include <memory>
#include "Intersection.h"   // Giả định bạn có lớp Node
#include "Road.h"   // Giả định bạn có lớp Road

class Graph {
public:
    TrafficGraph(const std::string& json_path);
    
    // Phương thức công khai
    std::shared_ptr<Road> get_edge(const std::string& edge_name);

private:
    std::map<std::string, std::shared_ptr<Node>> nodes;
    std::map<std::string, std::shared_ptr<Road>> edges;
    std::map<std::string, std::string> buildings; // Lưu tên tòa nhà và nút liên quan

    void load_from_json(const std::string& json_path);
    void load_edges(const std::map<std::string, dynamic_data>& data);
    void create_directional_road(const std::string& edge_id, 
                                 std::shared_ptr<Node> source, 
                                 std::shared_ptr<Node> target, 
                                 int lanes, double speed_limit, bool is_one_way);
};

#endif // TRAFFIC_GRAPH_H
