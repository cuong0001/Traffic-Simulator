#ifndef GRAPH_H
#define GRAPH_H
#include <vector>
#include <map>
#include "intersection.h"
#include "road.h"

class graph {
private:
    std::map<std::string, intersection> nodes;
    std::vector<road> roads;
public:
    void add_node(const intersection& node);
    void add_road(const road& road);
};

#endif
