#ifndef PATHFINDING_STRATEGY_H
#define PATHFINDING_STRATEGY_H
#include <vector>
#include <string>
class path_finding_strategy {
public:
    virtual ~path_finding_strategy() = default;
    virtual std::vector<std::string> calculate_path(std::string start, std::string end) = 0;
};

#endif
