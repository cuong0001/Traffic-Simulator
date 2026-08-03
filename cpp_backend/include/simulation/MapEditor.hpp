#pragma once
/*
MapEditor.hpp

- Owns live map editing: adding/removing junctions, roads and buildings,
  splitting a road under traffic, and re-tuning an existing road or light.
  Every method here assumes the caller (TrafficSimulator) already holds its
  simulation mutex -- this class has no lock of its own, the same convention
  TrafficSimulator's own "_locked" helpers already use.

- Split out of TrafficSimulator so structural graph edits have a class
  boundary of their own, matching the assignment's suggested component
  split. It is handed references to the graph pointer, the vehicle fleet and
  the event controller rather than owning any of them: editing the map can
  despawn vehicles standing on what got deleted and has to ask survivors to
  replan, which needs the same fleet TrafficSimulator drives every tick.
*/

#include <memory>
#include <string>
#include <vector>

class TrafficGraph;
class TrafficController;
class Vehicle;

class MapEditor {
public:
    MapEditor(TrafficGraph*& graph, std::vector<std::unique_ptr<Vehicle>>& vehicles,
             TrafficController& controller, int& next_id);

    // Refresh the node/road id caches used by random_node()-style lookups and
    // TrafficGraph's own per-node/per-road indices. Call after any edit that
    // adds or removes an Intersection or Road, and once after load().
    void reindex();

    // The graph itself changed under their wheels: ask every vehicle to
    // replan on the next tick.
    void invalidate_routes();

    // Each returns the new id, or "" on failure. Vehicles on a removed
    // element are despawned and routed vehicles are asked to replan.
    std::string add_junction_at(double x, double y);
    std::string split_road_near(double x, double y);       // create new junction id
    std::string add_road_between(const std::string& from, const std::string& to,
                                 double speed_kmh, int lanes, bool oneway);
    bool remove_road(const std::string& id);
    bool remove_intersection(const std::string& id);
    bool update_road(const std::string& id, double speed_kmh, bool closed);
    bool set_light(const std::string& node_id, bool on, double green);
    std::string add_building_at(double x, double y, const std::string& kind,
                                const std::string& name, double size);
    bool remove_building(const std::string& name);

    // Cached ids, refreshed by reindex(); used for random spawn/trip endpoints.
    const std::vector<std::string>& node_ids() const { return node_ids_; }
    const std::vector<std::string>& road_ids() const { return road_ids_; }

private:
    // Detaches a vehicle from its road and the event Subject, but does not
    // free it -- the caller erases the corresponding entry from vehicles_
    // right after, same convention as TrafficSimulator::destroy_vehicle_locked.
    void destroy_vehicle(Vehicle* v);

    // Drop a road and its reverse twin, detaching any vehicles on them.
    bool remove_road_locked(const std::string& id);

    TrafficGraph*& graph_;
    std::vector<std::unique_ptr<Vehicle>>& vehicles_;
    TrafficController& controller_;
    int& next_id_;

    std::vector<std::string> node_ids_;
    std::vector<std::string> road_ids_;
};
