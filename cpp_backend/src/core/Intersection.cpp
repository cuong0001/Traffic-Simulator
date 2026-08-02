#include "core/Intersection.hpp"
#include "core/Road.hpp"
#include <utility>

Intersection::Intersection(std::string name, double x, double y)
    : name(std::move(name)), x(x), y(y),
      is_building(false), is_roundabout(false), is_signalized(false),
      radius(0.0), light_system(nullptr) {
    display_name = this->name;  // default the label to the id
}

bool Intersection::is_intersection() const {
    return !is_building && incoming_edges.size() > 1;
}

// A junction with no light is green to everyone: callers must not have to
// special-case unsignalized nodes to let traffic through.
unsigned int Intersection::signal_state_for(int group, double sim_time) const {
    if (!is_signalized || !light_system) return TrafficLight::Green;
    return light_system->state_at(group, sim_time);
}

double Intersection::signal_remaining_for(int group, double sim_time) const {
    if (!is_signalized || !light_system) return 0.0;
    return light_system->remaining_at(group, sim_time);
}
