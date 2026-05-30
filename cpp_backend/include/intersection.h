#ifndef INTERSECTION_H
#define INTERSECTION_H
#include <string>

class intersection {
private:
    std::string id;
    double x, y;
    bool has_traffic_light;
public:
    intersection(std::string id, double x, double y, bool traffic_light = false);
    std::string get_id() const { return id; }
};

#endif
