#ifndef INTERSECTION_H
#define INTERSECTION_H
#include <string>

class Intersection {
private:
    std::string id;
    double x, y;
    bool hasTrafficLight;
public:
    Intersection(std::string id, double x, double y, bool trafficLight = false);
    std::string getId() const { return id; }
};

#endif
