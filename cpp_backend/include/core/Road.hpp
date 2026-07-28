#pragma once
#include <string>
#include <vector>
#include <utility>

class Intersection;
class Vehicle; 
class Geometry; 

class Road {
public:

    std::string id;

    Intersection* source;
    Intersection* target;

    int lanes;
    double speed_limit;
    int capacity;           //Max number of vehicles this road holds before it is jammed.
    bool is_one_way;        //A two-way street is stored as two Road objects ("id" and "id_r").
    bool is_internal;       //Check if this road belongs to the roundabout.
    double base_weight;     //Travel cost of this road in ideal conditions.
    double congestion_level;//Measuring traffic density of the road
    double travel_cost;    //Travel time based on congestion level.
    double length;         //The length of the road.

    Geometry* geometry;     //Simulate the road using a function.

    //List of vehicles moving on this road.
    std::vector<Vehicle*> vehicles;

    //Attributes for TraficEvent:
    bool is_closed = false;         // use when that event cause close road
    double event_congestion = 0.0;  // congestion level caused by traffic event
    double capacity_scale = 1.0;    // Capacity ratio following a traffic event
    static constexpr double kClosedRoadCost = 1e8;

    
    //A length <= 0 means "measure it from the endpoints".
    Road(std::string id, Intersection* source, Intersection* target, int lanes, double speed_limit,
        int capacity = 12, bool is_one_way = false, double length = 0.0,
        Geometry* geometry = nullptr);
        
    //Suggest behaviors
    void add_vehicle(Vehicle* vehicle);
    void remove_vehicle(Vehicle* vehicle);
    double get_congestion_level() const; //If congesttion level is high, vehicles preparing to enter this road will change to another route
    double get_weight() const;
    void update();
    
    // Check to see if there are any vehicles or obstacles ahead.
    std::pair<Vehicle*, double> get_lead_vehicle(Vehicle* current_vehicle); //return ahead vehicles or obstacles and distance
    
    //Behaviour for TraficEvent:
    void update_travel_cost();      // Use for update travel cost
};
