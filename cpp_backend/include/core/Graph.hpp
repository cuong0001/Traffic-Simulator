#pragma once
#include <string>
#include <unordered_map>
#include <vector>
#include "core/Zone.hpp"

// Use forward declarations instead of #include to speed up builds and avoid circular dependencies
class Intersection;
class Road;
class Building;

class TrafficGraph {
public:
    // Store objects in unordered_maps keyed by name/ID -> object pointer
    std::unordered_map<std::string, Intersection*> intersections;
    std::unordered_map<std::string, Road*> edges;
    std::unordered_map<std::string, Building*> buildings;
    std::vector<Zone> zones; // Display-only background regions, loaded from the GeoJSON map

    // Construct an empty graph, then populate it with MapLoader
    TrafficGraph() = default;
    // Destroy the graph, freeing all Intersection, Road, and Building memory
    ~TrafficGraph();

    // Graph construction
    void create_directional_road(const std::string& edge_id, Intersection* source_node, Intersection* target_node, int lanes, double speed_limit, bool is_one_way);

    // Lookup
    Road* get_edge(const std::string& edge_name) const;

    // Put a signal on a junction and hand every incoming road a phase group.
    // Approaches are clustered by BEARING into two opposing axes, so the two
    // streams that actually cross take turns instead of the whole junction
    // opening and closing together.
    //
    // Refuses (and returns false) on a roundabout, a waypoint, a junction with
    // fewer than 3 approaches, or one whose approaches all lie on a single
    // axis -- none of those has two streams to arbitrate, and signalizing them
    // just stops traffic for no reason.
    // Hand every incoming road of `node_id` a phase group and a stop line.
    // Returns HOW MANY distinct groups were assigned:
    //
    //   0  refused: a roundabout, a waypoint, or a node with no approaches
    //   1  every approach lies on one axis, so they all share group 0 and the
    //      junction simply cycles for everyone -- there is no crossing stream
    //      to arbitrate, but nothing is held on red either
    //   2  two opposing axes, which is the case a signal actually exists for
    //
    // Callers that GUESS a signal should insist on 2. Callers loading a signal
    // a map file states explicitly should accept 1, or saving and reloading a
    // map would silently drop the light the author put there.
    int assign_signal_groups(const std::string& node_id);

    // assign_signal_groups() + a fresh light. Refuses unless the junction has
    // two opposing axes, so guessed signals never stop traffic for nothing.
    bool signalize(const std::string& node_id, double green = 25.0);

    // Remove the signal and clear the groups off its approaches.
    void designalize(const std::string& node_id);
}; 
