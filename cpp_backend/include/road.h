#ifndef ROAD_H
#define ROAD_H
#include <string>

class Road {
private:
    std::string id, startNodeId, endNodeId;
    double length, speedLimit;
public:
    Road(std::string id, std::string start, std::string end, double len, double limit);
    double getCost() const; // Tính trọng số đường cho thuật toán tìm đường
};

#endif
