#pragma once
#include <stdexcept>
#include <string>

class TrafficGraph;

// Error while reading a map file (cannot open, malformed JSON, missing a
// required property, and so on).
class MapLoadError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

/**
 * Load a map from a SINGLE GeoJSON file.
 *
 * Format: a GeoJSON FeatureCollection where every feature has a "uts:kind"
 * property saying what it is (coordinates are already in world meters):
 *
 *   - "node"     Point       {id, name, zone, [signal, sgroups, sgreen, samber, sred]}
 *   - "road"     LineString  {id, from, to, [length, speed_kmh, capacity, lanes, oneway]}
 *   - "zone"     Polygon     {name, kind}          - display-only background region
 *   - "building" Polygon     {name, kind, levels, [node]}  - building with a footprint;
 *                            optional "node" anchors the building to a node as a destination
 *
 * A two-way street is stored as ONE feature (oneway=false) and expands into
 * two Road objects: "id" and "id_r" (the reverse direction).
 */
class MapLoader {
public:
    // Load a map from a GeoJSON file into an empty graph. Throws MapLoadError.
    static void load_from_geojson(TrafficGraph& graph, const std::string& path);

    // Same, but takes the JSON text directly (used by unit tests).
    // source_name only appears in error messages.
    static void load_from_geojson_string(TrafficGraph& graph, const std::string& text,
                                         const std::string& source_name = "<string>");

    // Post-load check: returns a warning string if the graph is empty or has
    // nodes unreachable from the first node; returns "" if the graph is fine.
    static std::string validate(const TrafficGraph& graph);

    // Serialize the loaded graph to a compact JSON the web UI can draw, and
    // write it to `path`. This is the static map the frontend fetches once:
    //   { intersections:[{id,x,y,zone,signal}],
    //     roads:[{id,from,to,length,lanes,speed_kmh,oneway}],
    //     buildings:[{name,kind,levels,footprint:[[x,y],...]}],
    //     zones:[{name,kind,x,y,w,h}] }
    // Two-way roads are emitted once (the "_r" reverse twin is skipped).
    // Throws MapLoadError if the file cannot be written.
    static void write_map_json(const TrafficGraph& graph, const std::string& path);

    // Write the live graph back out in the native GeoJSON format, so a city
    // edited at runtime can be reloaded by load_from_geojson().
    // Throws MapLoadError if the file cannot be written.
    static void write_geojson(const TrafficGraph& graph, const std::string& path);

    // Same serialization, returned as a compact JSON string (served by GET /api/map).
    static std::string map_json_string(const TrafficGraph& graph);
};
