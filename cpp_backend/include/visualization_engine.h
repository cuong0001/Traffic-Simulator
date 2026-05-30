#ifndef VISUALIZATION_ENGINE_H
#define VISUALIZATION_ENGINE_H
#include <string>
#include <vector>
#include "vehicle.h"

class visualization_engine {
public:
    static std::string serialize_state(const std::vector<vehicle*>& v);
};
#endif
