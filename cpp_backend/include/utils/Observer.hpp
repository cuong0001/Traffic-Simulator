// This is Observer design pattern implementation for traffic simulation. 
// It allows vehicles to subscribe to traffic updates and receive notifications about changes in the traffic network, such as road closures, accidents, congestion, and traffic light status changes. 
// The Subject class manages a list of observers and broadcasts notifications to them when relevant events occur.
#pragma once

#include <algorithm>
#include <string>
#include <vector>

struct TrafficNotification /// Payload describing a change in the traffic network.
{
    enum class Kind {
        RoadClosed,
        RoadReopened,
        Accident,
        AccidentCleared,
        Congestion,
        CongestionCleared,
        TrafficLight,
        TrafficLightRestored
    };
    Kind kind;
    std::string roadId;          ///< affected road ("" if not road-scoped)
    std::string intersectionId;  ///< affected intersection ("" if not applicable)
    std::string description;     ///< human readable text for logs / UI
};

/// Interface implemented by anything that wants traffic updates (vehicles).
class IObserver {
public:
    virtual ~IObserver() = default;
    virtual void onNotify(const TrafficNotification& notification) = 0;
};

/// Subject base class: keeps a list of observers and broadcasts to them.
class Subject {
private:
    std::vector<IObserver*> observers_; // List of observers (vehicles) followed to traffic updates.
public:
    virtual ~Subject() = default;
    void attach(IObserver* observer); // Adds an observer to the list of observers if it is not already present.
    void detach(IObserver* observer); // Removes an observer from the list of observers if it is present.
    void notifyAll(const TrafficNotification& notification); // Notifies all observers in the list of observers about a change in the traffic network by calling their onNotify method with the provided notification.
};