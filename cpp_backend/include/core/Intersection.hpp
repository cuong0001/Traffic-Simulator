#pragma once
/**
 * Intersection.hpp
 * ----------------
 * An Intersection is a node of the road-network graph.
 * Each intersection has a unique identifier, a position (used by the
 * A* heuristic and the visualization layer), lists of incoming and
 * outgoing roads, and a simple traffic-light cycle.
 */

#include <cmath>
#include <string>
#include <vector>

#include "core/TrafficLight.hpp"
#include "core/Types.hpp"

namespace uts {

class Intersection {
public:
    Intersection(std::string id, node_id_t index, std::string name, double x, double y,
                 std::string zone = "");

    // --- identity / geometry ---------------------------------------------
    const std::string& id() const;
    node_id_t index() const;
    const std::string& name() const;
    const std::string& zone() const;
    /// Roundabout nodes (zone column = "roundabout"): no traffic light;
    /// vehicles circulate on a ring and entering traffic yields to it.
    bool isRoundabout();
    /// Waypoint nodes (zone = "waypoint"): degree-2 shape points inserted by
    /// the GeoJSON loader so roads follow real street curves. Pass-through
    /// only: no traffic light, never a trip origin/destination, not drawn.
    bool isWaypoint();
    double x() const;
    double y() const;

    double distanceTo(const Intersection& other) const;

    // --- connectivity ------------------------------------------------------
    void addOutgoing(road_id_t roadId);
    void addIncoming(road_id_t roadId);
    void removeRoad(road_id_t roadId);

    const std::vector<road_id_t>& outgoing() const;
    const std::vector<road_id_t>& incoming() const;

    // --- traffic signals ------------------------------------------------------
    /**
     * Per-APPROACH signals (Treiber / traffic-simulation.de style): the
     * junction owns the CYCLE, the approach roads carry the stop lines
     * (see core/TrafficLight.hpp). Opposing phase groups take turns:
     *
     *   group g: green [g*S, g*S+G) -> amber [+A) -> all-red [+R),
     *   where S = G+A+R; every other group is red throughout g's slot.
     *
     * The state is a pure function of sim time (stateless), and a per-id
     * phase offset keeps the whole city from switching in unison.
     * A TrafficLightEvent temporarily shrinks the green duration.
     */
    bool signalized() const;
    void setSignalized(bool on);
    int signalGroups() const;
    void setSignalGroups(int groups);
    double greenDuration() const;
    double amberDuration() const;
    double allRedDuration() const;
    void setCycle(double green, double amber, double allRed);
    SignalState signalStateFor(int group, double simTimeSeconds) const;

private:
    std::string id_;
    node_id_t index_;
    std::string name_;
    std::string zone_;
    double x_;
    double y_;

    std::vector<road_id_t> outgoing_;
    std::vector<road_id_t> incoming_;

    bool signalized_ = false;     ///< junction runs a signal cycle
    int signalGroups_ = 2;        ///< number of phase groups in the cycle
    double greenDur_ = 25.0;      ///< seconds of green per group
    double amberDur_ = 3.0;       ///< seconds of amber per group
    double allRedDur_ = 2.0;      ///< clearance interval per group
    double phaseOffset_ = 0.0;    ///< derived from the id hash
};

}  // namespace uts
