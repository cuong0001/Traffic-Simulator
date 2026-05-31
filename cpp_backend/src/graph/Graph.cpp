#include "traffic_graph.h"
#include <iostream>
#include <cmath>

TrafficGraph::TrafficGraph(const std::string& json_path) {
    load_from_json(json_path);
}

void TrafficGraph::load_from_json(const std::string& json_path) {
    // Tại đây bạn thực hiện đọc file JSON (sử dụng thư viện như nlohmann/json)
    // Sau đó khởi tạo các Node và Buildings như logic đã mô tả
    std::cout << "Loading graph from: " << json_path << std::endl;
}

void TrafficGraph::create_directional_road(const std::string& edge_id, 
                                           std::shared_ptr<Node> source, 
                                           std::shared_ptr<Node> target, 
                                           int lanes, double speed_limit, bool is_one_way) {
    // Logic tính toán hình học Cubic Bezier
    double dx = target->x - source->x;
    double dy = target->y - source->y;
    double length = std::hypot(dx, dy);

    if (length == 0) return;

    // Khởi tạo đối tượng Road mới và thêm vào bản đồ
    auto new_road = std::make_shared<Road>(edge_id, source, target, lanes, speed_limit);
    edges[edge_id] = new_road;
    
    source->outgoing_edges.push_back(new_road);
    target->incoming_edges.push_back(new_road);
}

void TrafficGraph::load_edges(const std::map<std::string, dynamic_data>& data) {
    // Logic vòng lặp duyệt qua các edges và gọi create_directional_road
}

std::shared_ptr<Road> TrafficGraph::get_edge(const std::string& edge_name) {
    if (edges.find(edge_name) != edges.end()) {
        return edges[edge_name];
    }
    return nullptr;
}
