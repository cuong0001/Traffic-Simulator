#pragma once
#include <memory>
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
    // Owning containers keyed by name/ID. unique_ptr means the graph's
    // destructor is compiler-generated (Rule of Zero) instead of a manual
    // delete loop, and a missing entry can never be a dangling pointer that
    // *looks* like it might still be valid.
    std::unordered_map<std::string, std::unique_ptr<Intersection>> intersections;
    std::unordered_map<std::string, std::unique_ptr<Road>> edges;
    std::unordered_map<std::string, std::unique_ptr<Building>> buildings;
    std::vector<Zone> zones; // Display-only background regions, loaded from the GeoJSON map

    // Construct an empty graph, then populate it with MapLoader.
    //
    // All four of these are declared here but defined "= default" in
    // Graph.cpp, not inline. Intersection/Road/Building are only
    // forward-declared in this header, and a unique_ptr<T> member needs T's
    // complete definition (to call delete) not just for the destructor, but
    // for the default constructor too: computing whether TrafficGraph()
    // is noexcept requires checking whether unordered_map<string,
    // unique_ptr<Building>>'s own default constructor is noexcept, which
    // (at least in libstdc++) instantiates that map's destructor along the
    // way. Defining all four after Graph.cpp's own #includes gives the
    // compiler the complete types it needs; defining any of them here would
    // not compile.
    TrafficGraph();
    ~TrafficGraph();

    // Owns every Intersection/Road/Building via unique_ptr, so copying would
    // either double-free (shallow copy) or require deep-cloning a graph,
    // which nothing needs. Move is fine and used by load()/import_map().
    TrafficGraph(const TrafficGraph&) = delete;
    TrafficGraph& operator=(const TrafficGraph&) = delete;
    TrafficGraph(TrafficGraph&&);
    TrafficGraph& operator=(TrafficGraph&&);

    // Graph construction
    void create_directional_road(const std::string& edge_id, Intersection* source_node, Intersection* target_node, int lanes, double speed_limit, bool is_one_way);

    // Lookup
    Road* get_edge(const std::string& edge_name) const;

    // Assign dense integer ids: Intersection::index over 0..intersections.size()-1,
    // Road::index likewise over edges, and Road::base_index shared by a two-way
    // street's "id" and "id_r" twin. These back the O(1) lookups in
    // TrafficSimulator::solve_junctions() -- call this after ANY edit that adds
    // or removes an intersection or road (the loader calls it once after the
    // initial load; live map editing must call it again, or new elements keep
    // the default index 0 and silently alias whatever already owns that slot).
    void reindex();

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
