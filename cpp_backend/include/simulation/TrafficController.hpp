#pragma once
/**
 * TrafficController.hpp
 * ---------------------
 * Manages the lifecycle of traffic events and broadcasts notifications
 * to registered observers (vehicles). This class is the Subject of the
 * Observer design pattern.
 */

#include <memory>
#include <random>
#include <string>
#include <vector>

#include "core/Graph.hpp"
//#include "events/TrafficEvent.hpp"
#include "utils/Observer.hpp"

class TrafficController : public Subject {
public:
    explicit TrafficController(TrafficGraph& graph);

    /**
     * Trigger an event manually (also used by the HTTP API).
     * @param type     congestion | accident | closure | light
     * @param targetId road id (or intersection id for "light");
     *                 empty string -> random suitable target
     * @param duration seconds; <= 0 -> sensible default per type
     * @return event id, or "" if it could not be created
     */
    std::string triggerEvent(const std::string& type, std::string targetId,
                             double duration);

    /// Advance event timers; expire and clean up finished events.
    void update(double dt);

    /// Force-expire every active event whose target is `targetId` (used by
    /// live map editing before a road is removed or split, so no event is
    /// left pointing at an id that no longer exists). Returns the count.
    int expireEventsTargeting(const std::string& targetId);

    /// Enable/disable random background events.
    void setRandomEvents(bool enabled) { randomEvents_ = enabled; }
    bool randomEvents() const { return randomEvents_; }

    const std::vector<std::unique_ptr<TrafficEvent>>& activeEvents() const {
        return events_;
    }

    /// Recent human-readable log lines for the UI.
    const std::vector<std::string>& log() const { return log_; }

    /// Let the simulator report noteworthy happenings (gridlock breaks,
    /// rescues) into the same UI event log.
    void logLine(const std::string& line) { pushLog(line); }

private:
    std::string randomRoad();
    std::string randomIntersection();
    void pushLog(const std::string& line);

    TrafficGraph& graph_;
    std::vector<std::unique_ptr<TrafficEvent>> events_;
    std::vector<std::string> log_;
    std::mt19937 rng_{20260703};
    bool randomEvents_ = true;
    double randomEventTimer_ = 25.0;  ///< seconds until the next random event
    int nextEventId_ = 1;
};