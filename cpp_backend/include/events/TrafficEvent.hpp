#pragma once
/*
 * TrafficEvent.hpp

 * Dynamic traffic events. Each event knows how to apply itself to the
 * graph, how to undo itself when it expires, and which notification it
 * emits (consumed by vehicles through the Observer pattern).
 
 * Class hierarchy (inheritance + polymorphism):
     TrafficEvent (abstract)
       ├── CongestionEvent   : extra travel cost on a road
       ├── AccidentEvent     : capacity collapse + heavy congestion
       ├── RoadClosureEvent  : road temporarily disabled
       └── TrafficLightEvent : longer red phase at an intersection
*/

#include <memory>
#include <string>

#include "core/Graph.hpp"
#include "utils/Observer.hpp"

class TrafficEvent {
public:
    TrafficEvent(std::string id, std::string targetId, double durationSeconds);
    virtual ~TrafficEvent() = default;

    const std::string& id() const { return id_; }
    const std::string& target() const { return targetId_; }
    double remaining() const { return durationSeconds_ - elapsed_; }
    double duration() const { return durationSeconds_; }
    bool expired() const { return elapsed_ >= durationSeconds_; }

    virtual std::string type() const = 0;
    virtual std::string description() const = 0;

    /// Mutate the graph when the event starts.
    virtual void apply(TrafficGraph& graph) = 0;
    /// Restore the graph when the event ends.
    virtual void expire(TrafficGraph& graph) = 0;

    /// Notifications broadcast to observers on start / end.
    virtual TrafficNotification startNotification() const = 0;
    virtual TrafficNotification endNotification() const = 0;

    void advance(double dt) { elapsed_ += dt; }

protected:
    std::string id_;
    std::string targetId_;
    double durationSeconds_;
    double elapsed_ = 0.0;
};

/// Traffic density spike: raises travel cost on one road.
class CongestionEvent : public TrafficEvent {
public:
    CongestionEvent(std::string id, std::string roadId, double durationSeconds,
                    double intensity = 0.5);
    std::string type() const override { return "congestion"; }
    std::string description() const override;
    void apply(TrafficGraph& graph) override;
    void expire(TrafficGraph& graph) override;
    TrafficNotification startNotification() const override;
    TrafficNotification endNotification() const override;

private:
    double intensity_;
};

/// Accident: road capacity collapses and congestion soars.
class AccidentEvent : public TrafficEvent {
public:
    AccidentEvent(std::string id, std::string roadId, double durationSeconds);
    std::string type() const override { return "accident"; }
    std::string description() const override;
    void apply(TrafficGraph& graph) override;
    void expire(TrafficGraph& graph) override;
    TrafficNotification startNotification() const override;
    TrafficNotification endNotification() const override;
};

/// Full road closure: the edge becomes unusable.
class RoadClosureEvent : public TrafficEvent {
public:
    RoadClosureEvent(std::string id, std::string roadId, double durationSeconds);
    std::string type() const override { return "closure"; }
    std::string description() const override;
    void apply(TrafficGraph& graph) override;
    void expire(TrafficGraph& graph) override;
    TrafficNotification startNotification() const override;
    TrafficNotification endNotification() const override;
};

/// Broken / delayed traffic light: much shorter green phase.
class TrafficLightEvent : public TrafficEvent {
public:
    TrafficLightEvent(std::string id, std::string intersectionId,
                      double durationSeconds);
    std::string type() const override { return "light"; }
    std::string description() const override;
    void apply(TrafficGraph& graph) override;
    void expire(TrafficGraph& graph) override;
    TrafficNotification startNotification() const override;
    TrafficNotification endNotification() const override;

private:
    double previousGreen_ = 25.0;  ///< green DURATION (s) to restore on expiry
};

/// Simple factory for events (used by the controller and the HTTP API).
std::unique_ptr<TrafficEvent> makeEvent(const std::string& type,
                                        const std::string& id,
                                        const std::string& targetId,
                                        double durationSeconds);