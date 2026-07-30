#pragma once
/*
TrafficController.hpp

- Owns the live traffic events and is the Subject every Vehicle attaches to.

- Splitting this out of TrafficSimulator keeps two concerns apart: the
simulator moves vehicles, the controller decides what happens to the
network. It is also the natural place for the Observer subject, because the
things worth broadcasting -- a closure, an accident, a signal fault -- are
exactly the events it owns.

- The notifications are produced by the events themselves
(TrafficEvent::startNotification / endNotification), so there is one
description of what an event means, not two.
*/

#include <memory>
#include <random>
#include <string>
#include <vector>

#include "events/TrafficEvent.hpp"
#include "utils/Observer.hpp"

class TrafficGraph;

class TrafficController : public Subject {
public:
    TrafficController() = default;

    // Point the controller to a new graph (map), drops remaining events from the previous one.
    void set_graph(TrafficGraph* graph);

    // Start an event:
    //  - An empty target picks a random road or a random signalized junction for "light"
    //  - duration <= 0 uses a per-type default.
    // - Returns the new event id, or "" when the event could not be created.
    std::string trigger_event(const std::string& type, std::string target_id,
                              double duration);

    // - End every live event on `target_id` right now.
    // - Used before a road is deleted, so the graph is never left 
    // holding the effects of an event whose target has gone.
    int expire_events_targeting(const std::string& target_id);

    // Advance every event, expire the finished ones, and 
    // fire the occasional random event to keep the city alive.
    void update(double dt);

    // Drop every event without applying its undo. Only for a 
    // full reset, where the graph is being thrown away anyway.
    void clear();

    // introspection for the API and the UI
    const std::vector<std::unique_ptr<TrafficEvent>>& events() const
    { 
        return events_; 
    }

    const std::vector<std::string>& log() const 
    { 
        return log_; 
    }

    void log_line(const std::string& line); 

    void set_random_events(bool on) 
    {
        random_events_ = on; 
    }

    bool random_events() const 
    { 
        return random_events_; 
    }

private:
    std::string random_road() const;
    std::string random_signalized_node() const;

    TrafficGraph* graph_ = nullptr;
    std::vector<std::unique_ptr<TrafficEvent>> events_;
    std::vector<std::string> log_;
    int next_event_id_ = 1;

    bool random_events_ = true;
    double random_event_timer_ = 20.0;
    mutable std::mt19937 rng_{20260729u};  // fixed seed: replays are reproducible
};