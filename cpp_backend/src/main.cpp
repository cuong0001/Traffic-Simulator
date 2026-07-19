// Small demo: load a map from a GeoJSON file, export it as JSON for the web UI,
// and (optionally) print a Dijkstra route.
//
//   traffic_simulator [map.geojson] [start end]
//
// After loading, the map is written to web_frontend/map.json so the static web
// viewer can fetch and draw it (see web_frontend/js/api.js).
#include "routing/Dijkstra.hpp"
#include "core/Graph.hpp"
#include "utils/MapLoader.hpp"
#include "core/Intersection.hpp"
#include "core/Road.hpp"

#include <iostream>
#include <string>

int main(int argc, char** argv) {
    const std::string path =
        (argc > 1) ? argv[1] : "data/maps/small_map.geojson";

    TrafficGraph graph;
    try {
        MapLoader::load_from_geojson(graph, path);
    } catch (const MapLoadError& e) {
        std::cerr << "Map load error: " << e.what() << "\n";
        return 1;
    }

    std::cout << "Loaded '" << path << "':\n"
              << "  nodes     = " << graph.intersections.size() << "\n"
              << "  roads     = " << graph.edges.size() << "\n"
              << "  zones     = " << graph.zones.size() << "\n"
              << "  buildings = " << graph.buildings.size() << "\n";

    const std::string warning = MapLoader::validate(graph);
    std::cout << "  validate  = " << (warning.empty() ? "OK" : warning) << "\n";

    // Export the static map for the web UI to draw.
    const std::string map_out = "web_frontend/map.json";
    try {
        MapLoader::write_map_json(graph, map_out);
        std::cout << "  wrote     = " << map_out << "\n";
    } catch (const MapLoadError& e) {
        std::cerr << "Warning: could not write " << map_out << ": " << e.what() << "\n";
    }

    // Print one short route with Dijkstra when start/end node ids are given.
    if (argc > 3) {
        Dijkstra dijkstra;
        const std::vector<Intersection*> path_nodes = dijkstra.find_path(graph, argv[2], argv[3]);
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
