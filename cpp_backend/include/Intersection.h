#pragma once
#include <string>
#include <vector>
#include "TrafficLight.h" 
class Road;

class Intersection {
public:
    std::string name;         // Intersection id, used as the key in TrafficGraph
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

    TrafficLight* light_system;

    Intersection(std::string name, double x, double y);
    ~Intersection(); 

    bool is_intersection() const;
    void update(double dt); 
}; 
