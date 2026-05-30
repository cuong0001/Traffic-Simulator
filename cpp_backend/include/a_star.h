#ifndef A_STAR_H
#define A_STAR_H
#include "path_finding_strategy.h"
class a_star : public path_finding_strategy {
public:
    std::vector<std::string> calculate_path(std::string start, std::string end) override;
};
#endif
