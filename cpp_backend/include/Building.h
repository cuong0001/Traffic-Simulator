#pragma once
#include <string>

class Building {
public:
    std::string name;
    std::string node_id;
    std::string type;

    Building(std::string name, std::string node_id, std::string type);
};
