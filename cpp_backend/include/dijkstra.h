#ifndef DIJKSTRA_H
#define DIJKSTRA_H
#include "path_finding_strategy.h"
class dijkstra : public path_finding_strategy {
public:
    std::vector<std::string> calculate_path(std::string start, std::string end) override;
};

#endif
