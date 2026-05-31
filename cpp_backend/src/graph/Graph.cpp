#include "TrafficGraph.h"
#include <fstream>
#include <sstream>
#include <iostream>

// Hàm khởi tạo cũ sử dụng JSON đã được thay đổi để tránh lỗi nếu bạn không dùng JSON nữa
TrafficGraph::TrafficGraph(const std::string& node_path, const std::string& road_path, const std::string& building_path) {
    load_from_csv(node_path, road_path, building_path);
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

// Hàm nạp dữ liệu và xây dựng đồ thị từ 3 file CSV
void TrafficGraph::load_from_csv(const std::string& node_path, const std::string& road_path, const std::string& building_path) {
    std::string line, word;

    // ==========================================
    // 1. Đọc file nodes.csv (id, x, y)
    // ==========================================
    std::ifstream node_file(node_path);
    if (node_file.is_open()) {
        std::getline(node_file, line); // Bỏ qua dòng tiêu đề (header)
        
        while (std::getline(node_file, line)) {
            if (line.empty()) continue;
            std::stringstream ss(line);
            std::string id, x_str, y_str;

            if (std::getline(ss, id, ',') && 
                std::getline(ss, x_str, ',') && 
                std::getline(ss, y_str, ',')) {
                
                try {
                    double x = std::stod(x_str);
                    double y = std::stod(y_str);
                    
                    Node* new_node = new Node(id, x, y);
                    nodes[id] = new_node;
                } catch (const std::exception& e) {
                    std::cerr << "Lỗi chuyển đổi dữ liệu Node: " << e.what() << " tại dòng: " << line << std::endl;
                }
            }
        }
        node_file.close();
    } else {
        std::cerr << "Không thể mở file nodes.csv: " << node_path << std::endl;
    }

    // ==========================================
    // 2. Đọc file roads.csv (id, source, target, lanes, speed_limit, is_oneway)
    // ==========================================
    std::ifstream road_file(road_path);
    if (road_file.is_open()) {
        std::getline(road_file, line); // Bỏ qua dòng tiêu đề (header)
        
        while (std::getline(road_file, line)) {
            if (line.empty()) continue;
            std::stringstream ss(line);
            std::string edge_id, source_id, target_id, lanes_str, speed_str, is_oneway_str;

            if (std::getline(ss, edge_id, ',') &&
                std::getline(ss, source_id, ',') &&
                std::getline(ss, target_id, ',') &&
                std::getline(ss, lanes_str, ',') &&
                std::getline(ss, speed_str, ',') &&
                std::getline(ss, is_oneway_str, ',')) {
                
                try {
                    int lanes = std::stoi(lanes_str);
                    double speed_limit = std::stod(speed_str);
                    
                    // Xử lý giá trị bool (nhận diện cả dạng số "1"/"0" hoặc chữ "true"/"false")
                    bool is_one_way = (is_oneway_str == "1" || is_oneway_str == "true" || is_oneway_str == "TRUE");

                    // Chỉ tạo đường đi nếu cả 2 node source và target đều tồn tại
                    if (nodes.count(source_id) && nodes.count(target_id)) {
                        create_directional_road(edge_id, nodes[source_id], nodes[target_id], 
                                                lanes, speed_limit, is_one_way);
                    } else {
                        std::cerr << "Cảnh báo: Không tìm thấy Node cho cạnh " << edge_id << std::endl;
                    }
                } catch (const std::exception& e) {
                    std::cerr << "Lỗi chuyển đổi dữ liệu Road: " << e.what() << " tại dòng: " << line << std::endl;
                }
            }
        }
        road_file.close();
    } else {
        std::cerr << "Không thể mở file roads.csv: " << road_path << std::endl;
    }

    // ==========================================
    // 3. Đọc file buildings.csv (name, node_id, type)
    // ==========================================
    std::ifstream b_file(building_path);
    if (b_file.is_open()) {
        std::getline(b_file, line); // Bỏ qua dòng tiêu đề (header)
        
        while (std::getline(b_file, line)) {
            if (line.empty()) continue;
            std::stringstream ss(line);
            std::string building_name, node_id, type;

            if (std::getline(ss, building_name, ',') &&
                std::getline(ss, node_id, ',') &&
                std::getline(ss, type, ',')) {
                
                if (nodes.count(node_id)) {
                    Building* new_building = new Building(building_name, nodes[node_id], type);
                    buildings[building_name] = new_building;
                } else {
                    std::cerr << "Cảnh báo: Không tìm thấy Node liên kết cho Building " << building_name << std::endl;
                }
            }
        }
        b_file.close();
    } else {
        std::cerr << "Không thể mở file buildings.csv: " << building_path << std::endl;
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
