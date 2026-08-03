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
    bool reverse_twin = false; // true for the "id_r" half of a two-way pair --
                               // set by TrafficGraph::reindex() so callers on a
                               // hot path don't need to string-compare `id`.

    Intersection* source;
    Intersection* target;

    int capacity;           //Max number of vehicles this road holds before it is jammed.
    bool is_one_way;        //A two-way street is stored as two Road objects ("id" and "id_r").
    bool is_internal;       //Check if this road belongs to the roundabout.
    double base_weight;     //Travel cost of this road in ideal conditions.
    double travel_cost;    //Travel time based on congestion level -- see update_travel_cost().
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

    // Where OSM actually put the signal head for this approach, in world
    // coordinates. Only set when the source map really carried a
    // highway=traffic_signals node on this arm: signalizing a junction gives
    // EVERY approach a phase group so traffic stops correctly, but drawing a
    // head on every one of them invents lights the city does not have. The
    // renderer draws a head only where has_signal_head is true; the stopping
    // logic ignores this entirely and keeps using signal_stop_from_end.
    bool has_signal_head = false;
    double signal_head_x = 0.0, signal_head_y = 0.0;
    // Unit direction of travel of the road segment the head actually stands on,
    // pointing at the junction. The renderer needs the LOCAL heading to lay a
    // stop line across the carriageway; deriving it from the approach's two
    // endpoints instead gives the chord, which on a curved or split approach
    // crosses the road at a visible angle.
    double signal_head_ux = 0.0, signal_head_uy = 0.0;

    static constexpr double kClosedRoadCost = 1e8;


    //A length <= 0 means "measure it from the endpoints".
    Road(std::string id, Intersection* source, Intersection* target, int lanes, double speed_limit,
        int capacity = 12, bool is_one_way = false, double length = 0.0,
        Geometry* geometry = nullptr);
        
    //Suggest behaviors
    void add_vehicle(Vehicle* vehicle);
    void remove_vehicle(Vehicle* vehicle);
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
    double speed_limit_ms() const { return speed_limit_ / 3.6; }

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

    // ======================================================================
    // Encapsulated mutable state.
    //
    // These are the fields that actually change after construction --
    // TrafficEvent (accidents, closures, congestion) and live map editing
    // (update_road()) mutate them, each under an invariant worth protecting
    // (a valid range, or "closed roads are impassable"). Everything above
    // (id, source/target, capacity, length, ...) is topology set once at
    // construction and read constantly on the simulation hot path; wrapping
    // it in a getter that just returns the same value would not protect
    // anything, so it stays public data. Only these needed moving.
    // ======================================================================
    int get_lanes() const { return lanes_; }
    void set_lanes(int n) { lanes_ = n < 1 ? 1 : n; }

    double get_speed_limit() const { return speed_limit_; }  // km/h
    void set_speed_limit(double kmh) { speed_limit_ = kmh < 5.0 ? 5.0 : kmh; }

    bool is_closed() const { return is_closed_; }
    void set_closed(bool closed) { is_closed_ = closed; }

    // update_travel_cost() is the only thing that normally computes this
    // from vehicle density + event_congestion_; the setter exists for tests
    // and scripted scenarios that want to force a congestion level directly,
    // and still clamps to the same [0, 1] every reader assumes.
    double get_congestion_level() const { return congestion_level_; }
    void set_congestion_level(double c) {
        congestion_level_ = c < 0.0 ? 0.0 : (c > 1.0 ? 1.0 : c);
    }

    double get_capacity_scale() const { return capacity_scale_; }
    // Traffic events act multiplicatively (an accident cuts capacity by a
    // factor; clearing it divides back), so the operation Road exposes is
    // "scale", not "set" -- that is what lets two overlapping accidents
    // compose instead of the second clobbering the first's effect.
    void scale_capacity(double factor) {
        capacity_scale_ *= factor;
        if (capacity_scale_ > 1.0) capacity_scale_ = 1.0;
        if (capacity_scale_ < 0.01) capacity_scale_ = 0.01;  // never fully seized
    }

    double get_event_congestion() const { return event_congestion_; }
    // Additive for the same reason: two overlapping congestion events must
    // stack and unwind independently.
    void add_event_congestion(double delta) {
        event_congestion_ += delta;
        if (event_congestion_ < 0.0) event_congestion_ = 0.0;
    }

private:
    int lanes_;
    double speed_limit_;              // km/h
    double congestion_level_ = 0.0;   // [0, 1], see update_travel_cost()
    bool is_closed_ = false;
    double capacity_scale_ = 1.0;     // (0, 1], following an accident
    double event_congestion_ = 0.0;   // >= 0, following a congestion event
};
