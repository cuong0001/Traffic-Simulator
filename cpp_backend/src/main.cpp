// Urban Traffic Simulator — backend entry point.
//
//   traffic_simulator [map.geojson] [port]
//
// Loads a GeoJSON map, exports web_frontend/map.json for the static viewer,
// runs the simulation on a background thread, and serves the web UI + REST API
// (/api/map, /api/state, /api/spawn, /api/event, /api/pause, /api/resume,
//  /api/speed, /api/auto) over HTTP.
#include "simulation/TrafficSimulator.hpp"
#include "server/WebSocketServer.hpp"
#include "core/Graph.hpp"
#include "utils/MapLoader.hpp"

#include <chrono>
#include <iostream>
#include <string>
#include <thread>

int main(int argc, char** argv) {
    const std::string path = (argc > 1) ? argv[1] : "data/maps/small_map.geojson";
    const int port = (argc > 2) ? std::atoi(argv[2]) : 8080;

    TrafficSimulator& sim = TrafficSimulator::instance();
    try {
        sim.load(path);
    } catch (const MapLoadError& e) {
        std::cerr << "Map load error: " << e.what() << "\n";
        return 1;
    }

    // Keep the static viewer working: also dump the map to a file.
    try {
        MapLoader::write_map_json(*sim.graph(), "web_frontend/map.json");
    } catch (const std::exception& e) {
        std::cerr << "Warning: could not write web_frontend/map.json: " << e.what() << "\n";
    }

    std::cout << "Loaded '" << path << "':\n"
              << "  nodes     = " << sim.graph()->intersections.size() << "\n"
              << "  roads     = " << sim.graph()->edges.size() << "\n"
              << "  buildings = " << sim.graph()->buildings.size() << "\n";

    // A map that parses can still be unusable: empty, or split into pieces the
    // routers cannot cross. Say so at startup rather than letting it surface
    // later as vehicles that mysteriously fail to spawn.
    const std::string warning = MapLoader::validate(*sim.graph());
    std::cout << "  validate  = " << (warning.empty() ? "OK" : warning) << "\n";

    // Simulation thread: advance ~20x/second in real time. The speed factor
    // (set via /api/speed) scales simulated time inside tick().
    std::thread([&sim] {
        const double dt = 0.05;  // 50 ms per step
        while (true) {
            sim.tick(dt);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }).detach();

    // Publisher: rebuilds the /api/state payload 20x a second so serving it
    // costs a pointer copy instead of a lock on the simulation.
    sim.start_publishing();

    // REST API dispatch.
    auto api = [&sim](const std::string& p, const HttpServer::Params& params) -> std::string {
        auto get = [&](const std::string& key, const std::string& def = "") {
            auto it = params.find(key);
            return it == params.end() ? def : it->second;
        };
        if (p == "/api/map") return sim.map_json();
        if (p == "/api/state") {
            // Lock-free: served from the published snapshot, so a heavy tick
            // can never stall the browser. The direct call is only the cold
            // start, before the publisher has produced anything.
            if (auto snap = sim.state_snapshot()) return *snap;
            return sim.state_json();
        }
        if (p == "/api/spawn") {
            const std::string id = sim.spawn_vehicle(get("from"), get("to"), get("type", "car"));
            return "{\"id\":\"" + id + "\"}";
        }
        if (p == "/api/event") {
            const std::string durs = get("dur");
            const double dur = durs.empty() ? 0.0 : std::stod(durs);
            const std::string id = sim.trigger_event(get("type"), get("target"), dur);
            return "{\"id\":\"" + id + "\"}";
        }
        if (p == "/api/pause") {
            sim.set_paused(true);
            return "{\"ok\":true}";
        }
        if (p == "/api/resume") {
            sim.set_paused(false);
            return "{\"ok\":true}";
        }
        if (p == "/api/speed") {
            const std::string x = get("x");
            if (!x.empty()) sim.set_speed(std::stod(x));
            return "{\"ok\":true}";
        }
        if (p == "/api/auto" || p == "/api/randomEvents") {
            const std::string on = get("on");
            sim.set_auto_spawn(on == "1" || on == "true");
            return "{\"ok\":true}";
        }

        // ---- traffic generation tuning ------------------------------------
        auto num = [&](const std::string& key, double def) {
            const std::string v = get(key);
            if (v.empty()) return def;
            try { return std::stod(v); } catch (const std::exception&) { return def; }
        };
        const std::string ok_true = "{\"ok\":true}";

        if (p == "/api/target") {
            sim.set_target_vehicles(static_cast<int>(num("value", 120)));
            return ok_true;
        }
        if (p == "/api/spawnRate") {
            sim.set_spawn_rate(num("value", 1.0));
            return ok_true;
        }
        if (p == "/api/algorithm") {
            return sim.set_algorithm(get("name")) ? ok_true
                 : "{\"ok\":false,\"error\":\"unknown algorithm\"}";
        }

        // ---- vehicle operations -------------------------------------------
        if (p == "/api/vehicle/detail") return sim.vehicle_detail_json(get("id"));
        if (p == "/api/removeVehicle") {
            return sim.remove_vehicle(get("id")) ? ok_true
                 : "{\"ok\":false,\"error\":\"no such vehicle\"}";
        }
        if (p == "/api/clearVehicles") {
            return "{\"ok\":true,\"removed\":" + std::to_string(sim.clear_vehicles()) + "}";
        }
        if (p == "/api/vehicle") {
            // Either retarget the vehicle, change its routing strategy, or both.
            const std::string id = get("id");
            bool done = false;
            if (!get("algorithm").empty()) done |= sim.set_vehicle_algorithm(id, get("algorithm"));
            if (!get("to").empty())        done |= sim.retarget_vehicle(id, get("to"));
            return done ? ok_true : "{\"ok\":false,\"error\":\"vehicle update failed\"}";
        }

        // ---- live map editing ---------------------------------------------
        if (p == "/api/addRoad") {
            // An endpoint is a junction id, a point on a road (split it there),
            // or empty ground (create a junction). Resolving server-side by
            // coordinates means "both ends on the same road" also works: the
            // second projection already sees the post-split graph.
            auto endpoint = [&](const std::string& prefix) -> std::pair<std::string, bool> {
                const std::string kind = get(prefix + "Kind");
                if (kind == "road" || kind == "free") {
                    const double x = num(prefix + "X", 0.0), y = num(prefix + "Y", 0.0);
                    return kind == "road" ? std::make_pair(sim.split_road_near(x, y), false)
                                          : std::make_pair(sim.add_junction_at(x, y), true);
                }
                return {get(prefix), false};
            };
            auto [from_id, from_free] = endpoint("from");
            auto [to_id, to_free] = endpoint("to");
            if (from_id.empty() || to_id.empty())
                return "{\"ok\":false,\"error\":\"from and to endpoints are required\"}";

            // A brand-new dead end must stay leavable, so force two-way there.
            bool oneway = get("oneway") == "true";
            const bool forced = oneway && (from_free || to_free);
            if (forced) oneway = false;

            const std::string id = sim.add_road_between(
                from_id, to_id, num("speed", 40.0),
                static_cast<int>(num("lanes", 1)), oneway);
            if (id.empty()) return "{\"ok\":false,\"error\":\"could not create road\"}";
            return "{\"ok\":true,\"id\":\"" + id + "\",\"from\":\"" + from_id +
                   "\",\"to\":\"" + to_id + "\",\"forcedTwoWay\":" +
                   (forced ? "true" : "false") + "}";
        }
        if (p == "/api/removeRoad") {
            return sim.remove_road(get("id")) ? ok_true
                 : "{\"ok\":false,\"error\":\"no such road\"}";
        }
        if (p == "/api/removeIntersection") {
            return sim.remove_intersection(get("id")) ? ok_true
                 : "{\"ok\":false,\"error\":\"no such junction\"}";
        }
        if (p == "/api/updateRoad") {
            const bool closed = get("closed") == "true" || get("closed") == "1";
            return sim.update_road(get("id"), num("speed", 0.0), closed) ? ok_true
                 : "{\"ok\":false,\"error\":\"no such road\"}";
        }
        if (p == "/api/setLight") {
            const std::string on = get("on", "true");
            return sim.set_light(get("id"), on == "true" || on == "1", num("green", 25.0))
                 ? ok_true : "{\"ok\":false,\"error\":\"no such junction\"}";
        }
        if (p == "/api/addBuilding") {
            if (get("x").empty() || get("y").empty())
                return "{\"ok\":false,\"error\":\"x and y are required\"}";
            const std::string id = sim.add_building_at(num("x", 0.0), num("y", 0.0),
                                                       get("kind"), get("name"),
                                                       num("size", 24.0));
            return id.empty() ? "{\"ok\":false,\"error\":\"could not add building\"}"
                              : "{\"ok\":true,\"id\":\"" + id + "\"}";
        }
        if (p == "/api/removeBuilding") {
            return sim.remove_building(get("id")) ? ok_true
                 : "{\"ok\":false,\"error\":\"no such building\"}";
        }

        // ---- map files -----------------------------------------------------
        if (p == "/api/maps") return sim.maps_json();
        if (p == "/api/saveMap") {
            return sim.save_map(get("name")) ? ok_true
                 : "{\"ok\":false,\"error\":\"could not save map\"}";
        }
        if (p == "/api/loadMap") {
            return sim.load_map(get("name")) ? ok_true
                 : "{\"ok\":false,\"error\":\"could not load map\"}";
        }

        return "{\"error\":\"unknown endpoint\"}";
    };

    HttpServer server(port, "web_frontend", api);
    server.run();  // blocking accept loop
    return 0;
}
