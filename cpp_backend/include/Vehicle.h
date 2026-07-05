#pragma once
#include <string>
#include <vector>

class Intersection;
class Road;
class Building;
class TrafficGraph;

class Vehicle
{
public:
    std::string id;
    Road* current_road; //the road the vehicle is on
    Building* destination; //destination of vehicle
    double position_on_road; //the distance traveled by the vehicle, measured from the start of the road
    double current_speed; //current speed of vehicle
    double max_speed; //max speed of vehicle

    std::vector<Road*> route; //List of roads forming the route from the current location to the destination.
    int route_index; //route[route_index]=curent_road;
    std::vector<std::string> travel_history; //List of road id the vehicle has passed through
    bool is_reached_destination;

    //Constructor
    Vehicle(std::string id, double max_speed);

    //Destructor
    virtual ~Vehicle() = default;

    //Finding route from buiding A to building B
    void request_route(TrafficGraph* graph, Building* start, Building* target);

    //Update movement step-by-step based on delta time (dt).
    void step_by_step(double dt);

    //Detecting obstacles or traffic jam to decide whether to slow down, stop, or change the route
    bool react_to_congestion();

    //Refinding route from Intersection C to building B
    void recompute_route(TrafficGraph* graph, Intersection* current, Building* target);

    //Check if the vehicle has reached its destination.
    bool has_reached_destination() const;

    //getter
    std::string get_id() const; //get name of vehicle
    Road* get_current_road() const; //get current road of vehicle
    double get_position_on_road() const; //get the distance traveled by the vehicle, measured from the start of the road
    double get_current_speed() const; //get current speed of vehicle
    const std::vector<std::string>& get_travel_history() const; //get travel history of vehicle
    const std::vector<Road*>& get_route() const; //get route of vehicle
    int& get_route_index() const; //get current road vehicle's route
    
    //setter
    void set_destination(Building* dest) { destination = dest; }
};