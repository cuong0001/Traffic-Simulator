#pragma once
#include <string>
#include <vector>
#include <utility>
#include <cstdint>

class Intersection;
class Vehicle;
class Geometry;

// What a router is trying to minimise.
//
// Defined here, in core, rather than next to PathFinding: it is a property of
// how you price a ROAD, and Road::cost() is what actually answers it. Routing
// consumes it, so the dependency points core -> routing and not back.
//
//   Distance : shortest metres. Ignores traffic, so the answer is stable.
//   Time     : fastest right now, using each road's current congestion. Two
//              vehicles asking a second apart can get different routes.
enum class CostMode { Distance, Time };

class Road {
public:

    std::string id;
    uint32_t index = 0;       // Fast zero-overhead integer ID
    uint32_t base_index = 0;  // Shared ID for two-way street twin roads (ignores _r)

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

    // Traffic signal on THIS approach. The junction owns the cycle; the
    // approach road owns where its stop line is and which phase group it
    // belongs to, so opposing streams can show different colours.
    // signal_group < 0 means "no signal on this approach".
    int signal_group = -1;
    double signal_stop_from_end = 8.0;  // m before the road's end

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

    // ======================================================================
    // Speed in the units the physics actually wants.
    //
    // speed_limit is stored in KM/H because that is what the map file carries
    // ("speed_kmh"), what write_geojson emits, and what the road-editing panel
    // shows. Everything that integrates motion works in METRES PER SECOND.
    // Converting here, once, is the only way both stay true; doing it at the
    // call sites is how the two units got mixed up in the first place.
    // ======================================================================
    double speed_limit_ms() const { return speed_limit / 3.6; }

    // Free-flow speed reduced by how congested this road currently is.
    // `sensitivity` is 1.0 for ordinary traffic and lower for vehicles that
    // filter through it -- an ambulance is barely slowed by a queue.
    double effective_speed_ms(double sensitivity = 1.0) const;

    // How long traversing the whole road takes at the current congestion.
    double travel_time_seconds(double sensitivity = 1.0) const;

    // Where a vehicle must stop for a red light on this approach, measured
    // from the START of the road. Returns `length` when there is no signal,
    // so a caller can use it unconditionally.
    double stop_line_position() const;

    // Cost of traversing this road under a routing policy. A closed road is
    // never traversable, so it costs infinity rather than a large finite
    // number -- a big number is still cheaper than a long detour, which is how
    // a "closed" road ends up on a route anyway.
    double cost(CostMode mode = CostMode::Distance) const;
};
