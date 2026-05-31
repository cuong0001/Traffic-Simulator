#include "TrafficGraph.h"
#include <fstream>
#include <iostream>
#include "json.hpp" // Sử dụng file header đơn lẻ đã tải về

using json = nlohmann::json;

// Khởi tạo Graph bằng đường dẫn file JSON
TrafficGraph::TrafficGraph(const std::string& json_path) {
    load_from_json(json_path);
}

// Hủy Graph, giải phóng toàn bộ vùng nhớ
TrafficGraph::~TrafficGraph() {
    for (auto& pair : nodes) {
        delete pair.second;
    }
    nodes.clear();

    for (auto& pair : edges) {
        delete pair.second;
    }
    edges.clear();

    for (auto& pair : buildings) {
        delete pair.second;
    }
    buildings.clear();
}

// Hàm nạp dữ liệu và xây dựng đồ thị từ cấu trúc JSON, 
void TrafficGraph::load_from_json(const std::string& json_path) {
    std::ifstream file(json_path);
    if (!file.is_open()) {
        std::cerr << "Không thể mở file JSON: " << json_path << std::endl;
        return;
    }

    try {
        json data;
        file >> data;

        // 1. Đọc danh sách Nodes (Cấu trúc Object: "id": {"x": ..., "y": ...})
        if (data.contains("nodes") && data["nodes"].is_object()) {
            for (auto& [node_id, node_info] : data["nodes"].items()) {
                double x = node_info.value("x", 0.0);
                double y = node_info.value("y", 0.0);
                
                // Giả sử Node có constructor nhận (id, x, y) hoặc chỉ cần truyền thông tin phù hợp
                Node* new_node = new Node(node_id, x, y); 
                nodes[node_id] = new_node;
            }
        }

        // 2. Đọc danh sách Edges (Cấu trúc Object: "id": {"source": ..., "target": ..., "lanes": ..., "speed_limit": ...})
        if (data.contains("edges") && data["edges"].is_object()) {
            for (auto& [edge_id, edge_info] : data["edges"].items()) {
                std::string source_id = edge_info.value("source", "");
                std::string target_id = edge_info.value("target", "");
                int lanes = edge_info.value("lanes", 1);
                double speed_limit = edge_info.value("speed_limit", 50.0);
                
                // Vì file dữ liệu tách rõ "a_b" và "b_a" nên đây mặc định là đường 1 chiều trong đồ thị
                bool is_one_way = true; 

                if (nodes.count(source_id) && nodes.count(target_id)) {
                    create_directional_road(edge_id, nodes[source_id], nodes[target_id], 
                                            lanes, speed_limit, is_one_way);
                }
            }
        }

        // 3. Đọc danh sách Buildings (Cấu trúc Object: "id": {"node": ..., "type": ...})
        if (data.contains("buildings") && data["buildings"].is_object()) {
            for (auto& [building_name, b_info] : data["buildings"].items()) {
                std::string node_id = b_info.value("node", "");
                std::string type = b_info.value("type", "");
                
                if (nodes.count(node_id)) {
                    // Giả sử Building nhận (tên_tòa_nhà, con_trỏ_node_liên_kết, loại_tòa_nhà)
                    Building* new_building = new Building(building_name, nodes[node_id], type);
                    buildings[building_name] = new_building;
                }
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "Lỗi khi xử lý file JSON: " << e.what() << std::endl;
    }
}

// Xây dựng đường đi (cạnh có hướng)
void TrafficGraph::create_directional_road(std::string edge_id, Node* source_node, Node* target_node, 
                                           int lanes, double speed_limit, bool is_one_way) {
    Road* new_road = new Road(source_node, target_node, lanes, speed_limit, is_one_way);
    edges[edge_id] = new_road;

}

// Truy xuất Road*
Road* TrafficGraph::get_edge(const std::string& edge_name) const {
    auto it = edges.find(edge_name);
    if (it != edges.end()) {
        return it->second;
    }
    return nullptr; 
}
