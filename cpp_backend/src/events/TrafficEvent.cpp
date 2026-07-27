#include "events/TrafficEvent.hpp"

#include <utility>
#include "core\Road.hpp"
#include "core\Intersection.hpp"
#include "core\TrafficLight.hpp"

using Kind = TrafficNotification::Kind;

TrafficEvent::TrafficEvent(std::string id, std::string targetId,
                           double durationSeconds)
    : id_(std::move(id)),
      targetId_(std::move(targetId)),
      durationSeconds_(durationSeconds) {}

// -Congestion
CongestionEvent::CongestionEvent(std::string id, std::string roadId,
                                 double durationSeconds, double intensity)
    : TrafficEvent(std::move(id), std::move(roadId), durationSeconds),
      intensity_(intensity) {}

std::string CongestionEvent::description() const 
{
    return "Heavy congestion on road " + targetId_;
}

void CongestionEvent::apply(TrafficGraph& graph) 
{
    Road* r = graph.get_edge(targetId_);
    if (r) 
    {
        r->event_congestion += intensity_; // Increased traffic congestion
        r->update_travel_cost();           // Update travel cost
    }
}

void CongestionEvent::expire(TrafficGraph& graph) 
{
    Road* r = graph.get_edge(targetId_);
    if (r) 
    {
        r->event_congestion -= intensity_; 
        if (r->event_congestion < 0.0) r->event_congestion = 0.0; 
        r->update_travel_cost();
    }
}

TrafficNotification CongestionEvent::startNotification() const 
{
    return {Kind::Congestion, targetId_, "", description()};
}

TrafficNotification CongestionEvent::endNotification() const 
{
    return {Kind::CongestionCleared, targetId_, "", "Congestion cleared on " + targetId_};
}

// Accident
AccidentEvent::AccidentEvent(std::string id, std::string roadId,
                             double durationSeconds)
    : TrafficEvent(std::move(id), std::move(roadId), durationSeconds) {}

std::string AccidentEvent::description() const 
{
    return "Accident on road " + targetId_;
}

void AccidentEvent::apply(TrafficGraph& graph) 
{
    Road* r = graph.get_edge(targetId_);
    if (r) 
    {
        r->capacity_scale *= 0.3; // The accident reduced capacity by 70%.
        r->event_congestion += 0.7; // Add traffic jams
        r->update_travel_cost();
    }
}

void AccidentEvent::expire(TrafficGraph& graph) 
{
    Road* r = graph.get_edge(targetId_);
    if (r) {
        r->capacity_scale /= 0.3; // Restoring road capacity following the accident
        if (r->capacity_scale > 1.0) r->capacity_scale = 1.0;
        r->event_congestion -= 0.7;
        r->update_travel_cost();
    }
}

TrafficNotification AccidentEvent::startNotification() const 
{
    return {Kind::Accident, targetId_, "", description()};
}

TrafficNotification AccidentEvent::endNotification() const 
{
    return {Kind::AccidentCleared, targetId_, "", "Accident cleared on " + targetId_};
}

// Road closure
RoadClosureEvent::RoadClosureEvent(std::string id, std::string roadId,
                                   double durationSeconds)
    : TrafficEvent(std::move(id), std::move(roadId), durationSeconds) {}

std::string RoadClosureEvent::description() const 
{
    return "Road " + targetId_ + " closed";
}

void RoadClosureEvent::apply(TrafficGraph& graph) 
{
    Road* r = graph.get_edge(targetId_);
    if (r) 
    {
        r->is_closed = true;
        r->update_travel_cost(); 
    }
}

void RoadClosureEvent::expire(TrafficGraph& graph) 
{
    Road* r = graph.get_edge(targetId_);
    if (r) 
    {
        r->is_closed = false;
        r->update_travel_cost();
    }
}

TrafficNotification RoadClosureEvent::startNotification() const 
{
    return {Kind::RoadClosed, targetId_, "", description()};
}

TrafficNotification RoadClosureEvent::endNotification() const 
{
    return {Kind::RoadReopened, targetId_, "", "Road " + targetId_ + " reopened"};
}

// Traffic light
TrafficLightEvent::TrafficLightEvent(std::string id, std::string intersectionId,
                                     double durationSeconds)
    : TrafficEvent(std::move(id), std::move(intersectionId), durationSeconds) {}

std::string TrafficLightEvent::description() const 
{
    return "Traffic light delay at " + targetId_;
}

void TrafficLightEvent::apply(TrafficGraph& graph) 
{
    auto it = graph.intersections.find(targetId_);
    if (it != graph.intersections.end()) 
    {
        Intersection* node = it->second;
        
        // Check if this intersection has traffic lights.
        if (node && node->light_system) 
        {
            // Retain the previous green light duration.
            previousGreen_ = node->light_system->get_green_duration();
            
            // Forcing the green light to just 5 seconds (simulating a light malfunction)
            node->light_system->set_green_duration(5.0); 
        }
    }
}

void TrafficLightEvent::expire(TrafficGraph& graph) 
{
    auto it = graph.intersections.find(targetId_);
    if (it != graph.intersections.end()) 
    {
        Intersection* node = it->second;
        
        if (node && node->light_system) 
        {
            // Restore the green light duration to its original setting.
            node->light_system->set_green_duration(previousGreen_); 
        }
    }
}

TrafficNotification TrafficLightEvent::startNotification() const {
    return {Kind::TrafficLight, "", targetId_, description()};
}
TrafficNotification TrafficLightEvent::endNotification() const {
    return {Kind::TrafficLightRestored, "", targetId_,
            "Traffic light restored at " + targetId_};
}

// Factory
std::unique_ptr<TrafficEvent> makeEvent(const std::string& type,
                                        const std::string& id,
                                        const std::string& targetId,
                                        double durationSeconds) {
    if (type == "congestion") {
        return std::make_unique<CongestionEvent>(id, targetId, durationSeconds);
    }
    if (type == "accident") {
        return std::make_unique<AccidentEvent>(id, targetId, durationSeconds);
    }
    if (type == "closure") {
        return std::make_unique<RoadClosureEvent>(id, targetId, durationSeconds);
    }
    if (type == "light") {
        return std::make_unique<TrafficLightEvent>(id, targetId, durationSeconds);
    }
    return nullptr;
}                                     