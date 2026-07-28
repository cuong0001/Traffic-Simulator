#pragma once
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <cmath>

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

    // --- CÁC THUỘC TÍNH BỔ SUNG PHỤC VỤ MÔ HÌNH IDM & CHUYỂN LÀN ---
    double idmFactor_{1.0};           // Hệ số cá thể hóa tốc độ (0.8 .. 1.0)
    double lateralOffset_{0.0};       // Tọa độ lệch ngang hiện tại
    double targetLateralOffset_{0.0}; // Tọa độ lệch ngang mục tiêu
    int lane_{0};                     // Làn đường hiện tại
    double stuckTime_{0.0};           // Thời gian bị kẹt liên tục

    // Constructor
    Vehicle(std::string id, double max_speed, double max_accel, double max_decel);

    // Destructor
    virtual ~Vehicle() = default;

    // --- HÀM TÍNH TOÁN IDM CỐT LÕI (Thêm mới) ---
    static double idmAccel(double v, double v0, double vLead, double gap);
    double desiredSpeedOn(const Road& road) const;

    // --- VIRTUAL METHODS PHỤC VỤ ĐA HÌNH ---
    virtual std::string type() const { return "car"; }
    virtual double speedFactor() const { return 1.0; }
    virtual double lengthMeters() const { return 4.5; }
    virtual double widthMeters() const { return 1.8; }
    virtual double congestionSensitivity() const { return 1.0; }
    virtual bool ignoresTrafficLights() const { return false; }
    virtual double lanePoliteness() const { return 0.5; }

    virtual std::vector<int> allowedLanes(int totalLanes) const {
        std::vector<int> lanes;
        for (int i = 0; i < totalLanes; ++i) lanes.push_back(i);
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

    int get_lane() const { return lane_; }
    double get_lateral_offset() const { return lateralOffset_; }

    // Setter
    void set_destination(Intersection* dest) { destination = dest; }
};
