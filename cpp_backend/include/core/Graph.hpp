#pragma once
#include <string>
#include <unordered_map>
#include <vector>
#include "core/Zone.hpp"

// Dùng Forward Declaration thay vì #include để tối ưu thời gian Build và tránh lỗi Circular Dependency
class Intersection;
class Road;
class Building;

class TrafficGraph {
public:
    // Dùng unordered_map lưu trữ theo dạng (Tên/ID -> Con trỏ đối tượng)
    std::unordered_map<std::string, Intersection*> nodes;
    std::unordered_map<std::string, Road*> edges;
    std::unordered_map<std::string, Building*> buildings;
    std::vector<Zone> zones; // Display-only background regions, loaded from the GeoJSON map

    // Construct an empty graph, then populate it with MapLoader
    TrafficGraph() = default;
    // Khởi tạo Graph bằng đường dẫn file csv
    TrafficGraph(const std::string& node_path, const std::string& road_path, const std::string& building_path);
    // Hủy Graph, giải phóng toàn bộ vùng nhớ của Intersection, Road, Building
    ~TrafficGraph();

    // Các hàm nạp dữ liệu và xây dựng đồ thị
    void load_from_csv(const std::string& node_path, const std::string& road_path, const std::string& building_path);
    void create_directional_road(const std::string& edge_id, Intersection* source_node, Intersection* target_node, int lanes, double speed_limit, bool is_one_way);

    // Truy xuất
    Road* get_edge(const std::string& edge_name) const;
}; 
