#ifndef ROAD_H
#define ROAD_H
#include <string>

class road {
private:
    std::string id, start_node_id, end_node_id;
    double length, speed_limit;
public:
    road(std::string id, std::string start, std::string end, double len, double limit);
    double get_cost() const; // Tính trọng số đường cho thuật toán tìm đường
};

#endif
