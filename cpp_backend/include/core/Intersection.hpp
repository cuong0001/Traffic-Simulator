#pragma once
#include <string>
#include <vector>
#include <cstdint>
#include <memory>
#include "core/TrafficLight.hpp"
class Road;

class Intersection {
public:
    std::string name;         // Intersection id, used as the key in TrafficGraph
    uint32_t index = 0;       // Fast zero-overhead integer ID
    std::string display_name; // Human-readable label, e.g. "Hillside & 1st"
    std::string zone;         // Region the node sits in ("", "downtown", "roundabout", ...)
    double x;
    double y;
    bool is_building;
    bool is_roundabout;
    bool is_signalized;       // Whether the node has a traffic light
    double radius;
    std::vector<Road*> outgoing_edges;
    std::vector<Road*> incoming_edges;

    std::unique_ptr<TrafficLight> light_system;

    Intersection(std::string name, double x, double y);
    ~Intersection() = default;

    // Owns light_system via unique_ptr; not meant to be copied (two
    // intersections should never share one light's ownership) and nothing
    // in the codebase copies one today.
    Intersection(const Intersection&) = delete;
    Intersection& operator=(const Intersection&) = delete;

    bool is_intersection() const;

    // Colour this junction shows to `group` at `sim_time`, and the seconds
    // left on it. Unsignalized junctions are green to everyone.
    unsigned int signal_state_for(int group, double sim_time) const;
    double signal_remaining_for(int group, double sim_time) const;
}; 
