// Urban Traffic Simulator — map server entry point.
//
//   traffic_simulator [map.geojson] [port]
//
// Loads a GeoJSON map, exports web_frontend/map.json (so the UI also works from
// a plain static file server), and serves the web UI + GET /api/map over HTTP.
// No simulation yet: this is the illustrated MAP. Vehicles, /api/state and the
// controls arrive in later PRs.
#include "core/Graph.hpp"
#include "server/WebSocketServer.hpp"
#include "utils/MapLoader.hpp"

#include <cstdlib>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    const std::string path = (argc > 1) ? argv[1] : "data/maps/small_map.geojson";
    const int port = (argc > 2) ? std::atoi(argv[2]) : 8080;

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
              << "  buildings = " << graph.buildings.size() << "\n";

    const std::string warning = MapLoader::validate(graph);
    std::cout << "  validate  = " << (warning.empty() ? "OK" : warning) << "\n";

    // Also dump map.json so the UI works without the server (python -m http.server).
    try {
        MapLoader::write_map_json(graph, "web_frontend/map.json");
    } catch (const std::exception& e) {
        std::cerr << "Warning: could not write web_frontend/map.json: " << e.what() << "\n";
    }

    // Serialize once; /api/map just hands back this string.
    const std::string map_json = MapLoader::map_json_string(graph);

    auto api = [&map_json](const std::string& p, const HttpServer::Params&) -> std::string {
        if (p == "/api/map") return map_json;
        return "{\"error\":\"unknown endpoint\"}";
    };

    HttpServer server(port, "web_frontend", api);
    server.run();  // blocking accept loop
    return 0;
}
