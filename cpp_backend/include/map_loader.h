#ifndef MAP_LOADER_H
#define MAP_LOADER_H
#include "graph.h"

class map_loader {
public:
    static void load_from_file(const std::string& path, graph& graph);
};
#endif
