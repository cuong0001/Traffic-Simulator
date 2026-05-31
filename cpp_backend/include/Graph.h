#pragma once
#include <string>
#include <unordered_map>

// Tại file Graph, chúng ta an toàn để include tất cả các header vì đây là cấp cao nhất
#include "Node.h"
#include "Road.h"
#include "Building.h"

class TrafficGraph {
public:
    // Dùng unordered_map lưu trữ theo dạng (Tên/ID -> Con trỏ đối tượng)
    std::unordered_map<std::string, Node*> nodes;
    std::unordered_map<std::string, Road*> edges;
    std::unordered_map<std::string, Building*> buildings;

    // Khởi tạo Graph bằng đường dẫn file JSON
    TrafficGraph(const std::string& node_path, const std::string& road_path, const std::string& building_path);
    // Hủy Graph, giải phóng toàn bộ vùng nhớ của Node, Road, Building
    ~TrafficGraph();

    // Các hàm nạp dữ liệu và xây dựng đồ thị
    void load_from_csv(const std::string& node_path, const std::string& road_path, const std::string& building_path);
    void create_directional_road(std::string edge_id, Node* source_node, Node* target_node, int lanes, double speed_limit, bool is_one_way);

    // Truy xuất
    Road* get_edge(const std::string& edge_name) const;
}; 
