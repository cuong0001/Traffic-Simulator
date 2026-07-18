#include "../../include/Node.h"
#include "../../include/Road.h"
#include <utility>

Node::Node(std::string name, double x, double y)
    : name(std::move(name)), x(x), y(y),
      is_building(false), is_roundabout(false), is_signalized(false),
      radius(0.0), light_system(nullptr) {
    display_name = this->name;  // default the label to the id
}

Node::~Node() {
    delete light_system;
}

bool Node::is_intersection() const {
    return !is_building && incoming_edges.size() > 1;
}

void Node::update(double dt) {
    if (light_system) {
        light_system->update(dt);
    }
}
