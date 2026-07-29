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
    Road* current_road; // The road the vehicle is on
    Intersection* destination; // Destination of vehicle
    double position_on_road; // The distance traveled by the vehicle, measured from the start of the road
    double current_speed; // Current speed of vehicle
    double max_speed; // Max speed of vehicle

    double accel;       // Current acceleration of vehicle
    double max_accel;   // Maximum acceleration of vehicle
    double max_decel;   // Maximum deceleration of vehicle

    std::vector<Road*> route; // List of roads forming the route from the current location to the destination.
    int route_index; // route[route_index]=current_road;
    std::vector<std::string> travel_history; // List of road id the vehicle has passed through
    bool is_reached_destination;

    // --- IDM and lane-change state ---
    double idm_factor{1.0};             // per-vehicle speed personality (0.8 .. 1.0)
    double lateral_offset{0.0};         // current sideways offset from the lane centre
    double target_lateral_offset{0.0};  // where the lane change is heading
    int lane{0};                        // current lane, 0 is the innermost
    double stuck_time{0.0};             // seconds spent standing still
    double last_gap{1e9};               // gap seen by the last react_to_congestion()

    // Constructor
    Vehicle(std::string id, double max_speed, double max_accel, double max_decel);

    // Destructor
    virtual ~Vehicle() = default;

    // --- core IDM maths ---
    static double idm_accel(double v, double v0, double v_lead, double gap);
    double desired_speed_on(const Road& road) const;

    // --- type hooks overridden by concrete vehicles (polymorphism) ---
    virtual std::string type() const = 0;
    virtual double speed_factor() const { return 1.0; }
    virtual double length_meters() const { return 4.5; }
    virtual double width_meters() const { return 1.8; }
    virtual double congestion_sensitivity() const { return 1.0; }
    virtual bool ignores_lights() const { return false; }
    virtual double lane_politeness() const { return 0.5; }

    virtual std::vector<int> allowed_lanes(int total_lanes) const {
        std::vector<int> lanes;
        for (int i = 0; i < total_lanes; ++i) lanes.push_back(i);
        return lanes;
    }

    // Finding route from node A to node B
    void request_route(TrafficGraph* graph, Intersection* start, Intersection* target);

    // Update movement step-by-step based on delta time (dt).
    void step_by_step(double dt);

    // Detecting obstacles or traffic jam to decide whether to slow down, stop, or change the route
    bool react_to_congestion();

    // Refinding route from node C to node B
    void recompute_route(TrafficGraph* graph, Intersection* target);

    // Check if the vehicle has reached its destination.
    bool has_reached_destination() const;

    // Getter
    std::string get_id() const; 
    Road* get_current_road() const; 
    double get_position_on_road() const; 
    const std::vector<std::string>& get_travel_history() const; 
    const std::vector<Road*>& get_route() const; 
    int get_route_index() const; 
    
    double get_current_speed() const; 
    double get_max_speed() const; 

    double get_acceleration() const; 
    double get_max_acceleration() const; 
    double get_max_deceleration() const; 

    int get_lane() const { return lane; }
    double get_lateral_offset() const { return lateral_offset; }

    // Setter
    void set_destination(Intersection* dest) { destination = dest; }
};
