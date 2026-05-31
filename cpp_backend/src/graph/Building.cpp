#include "../../include/Building.h"
#include <utility>

Building::Building(std::string name, std::string node_id, std::string type)
    : name(std::move(name)), node_id(std::move(node_id)), type(std::move(type)) {}
