#pragma once
#include <string>
#include <vector>
#include <utility>

class Node;
class Vehicle; 
class Geometry; 

class Road {
public:
    std::string name;
    
    Node* source;
    Node* target;
    
    int lanes;
    double speed_limit;
    bool is_internal;   //Check if this road belongs to the roundabout.
    double base_weight; //Travel cost of this road in ideal conditions.
    double weight;      //Real travel cost.
    
    Geometry* geometry; //Simulate the road using a function.

    //List of vehicles moving on this road.
    std::vector<Vehicle*> vehicles;

    Road(std::string name, Node* source, Node* target, int lanes, double speed_limit, Geometry* geometry = nullptr);

    //Suggest behaviors
    void add_vehicle(Vehicle* vehicle);
    void remove_vehicle(Vehicle* vehicle);
    double get_congestion_level() const; //If congesttion level is high, vehicles preparing to enter this road will change to another route
    void update();
    
    // Check to see if there are any vehicles or obstacles ahead.
    std::pair<Vehicle*, double> get_lead_vehicle(Vehicle* current_vehicle); //return ahead vehicles or obstacles and distance
};
