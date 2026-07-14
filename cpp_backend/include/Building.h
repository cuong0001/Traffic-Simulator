#pragma once
#include <string>
#include <utility>
#include <vector>

class Building {
public:
    std::string name;
    std::string node_id;  // Node mà tòa nhà gắn vào ("" nếu tòa nhà chỉ để hiển thị)
    std::string type;     // "commercial", "hospital", "residential", ...
    int levels;           // Số tầng (dùng khi vẽ)

    // Đường bao (polygon) của tòa nhà trong tọa độ thế giới.
    // Rỗng nếu tòa nhà chỉ được neo vào một node.
    std::vector<std::pair<double, double>> footprint;

    // Tòa nhà neo vào một node (định dạng CSV cũ)
    Building(std::string name, std::string node_id, std::string type);

    // Tòa nhà có đường bao lấy từ file map
    Building(std::string name, std::string type, int levels,
             std::vector<std::pair<double, double>> footprint);

    // Tâm của đường bao; nếu không có đường bao thì trả về (0, 0).
    std::pair<double, double> centroid() const;
};
