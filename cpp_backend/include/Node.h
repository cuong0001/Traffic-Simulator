#pragma once
#include <string>
#include <vector>
#include "TrafficLight.h" 
class Road;

class Node {
public:
    std::string name;
    double x;
    double y;
    bool is_building;
    bool is_roundabout;
    double radius;
    std::vector<Road*> outgoing_edges;
    std::vector<Road*> incoming_edges;

    TrafficLight* light_system; 

    Node(std::string name, double x, double y);
    ~Node(); 

    bool is_intersection() const;
    void update(double dt); 
}; 
