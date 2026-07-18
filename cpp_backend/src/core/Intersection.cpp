#include "core/Intersection.hpp"
#include "core/Road.hpp"
#include <utility>

Intersection::Intersection(std::string name, double x, double y)
    : name(std::move(name)), x(x), y(y),
      is_building(false), is_roundabout(false), is_signalized(false),
      radius(0.0), light_system(nullptr) {
    display_name = this->name;  // default the label to the id
}

Intersection::~Intersection() {
    delete light_system;
}

bool Intersection::is_intersection() const {
    return !is_building && incoming_edges.size() > 1;
}

void Intersection::update(double dt) {
    if (light_system) {
        light_system->update(dt);
    }
}
