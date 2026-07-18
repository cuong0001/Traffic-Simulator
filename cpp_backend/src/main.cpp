// Small demo: load a map from a GeoJSON file and print a summary plus one
// Dijkstra route.
#include "Dijkstra.h"
#include "Graph.h"
#include "MapLoader.h"
#include "Node.h"
#include "Road.h"

#include <iostream>
#include <string>

int main(int argc, char** argv) {
    const std::string path =
        (argc > 1) ? argv[1] : "data/maps/downtown.geojson";

    TrafficGraph graph;
    try {
        MapLoader::load_from_geojson(graph, path);
    } catch (const MapLoadError& e) {
        std::cerr << "Map load error: " << e.what() << "\n";
        return 1;
    }

    std::cout << "Loaded '" << path << "':\n"
              << "  nodes     = " << graph.nodes.size() << "\n"
              << "  roads     = " << graph.edges.size() << "\n"
              << "  zones     = " << graph.zones.size() << "\n"
              << "  buildings = " << graph.buildings.size() << "\n";

    const std::string warning = MapLoader::validate(graph);
    std::cout << "  validate  = " << (warning.empty() ? "OK" : warning) << "\n";

    // Print one short route with Dijkstra when start/end node ids are given.
    if (argc > 3) {
        Dijkstra dijkstra;
        const std::vector<Node*> path_nodes = dijkstra.find_path(graph, argv[2], argv[3]);
        std::cout << "\nDijkstra " << argv[2] << " -> " << argv[3] << ": ";
        if (path_nodes.empty()) {
            std::cout << "(no path)\n";
        } else {
            for (std::size_t i = 0; i < path_nodes.size(); ++i) {
                std::cout << path_nodes[i]->name << (i + 1 < path_nodes.size() ? " -> " : "\n");
            }
        }
    }
    return 0;
}
