#include "core/Intersection.hpp"

#include <algorithm>
#include <functional>
#include <utility>

namespace uts {

Intersection::Intersection(std::string id, node_id_t index, std::string name, double x, double y,
                           std::string zone)
    : id_(std::move(id)),
      index_(index),
      name_(std::move(name)),
      zone_(std::move(zone)),
      x_(x),
      y_(y) {
    // Deterministic pseudo-random phase so signals are out of sync.
    const std::size_t h = std::hash<std::string>{}(id_);
    phaseOffset_ = static_cast<double>(h % 1000) / 1000.0 * 60.0;
}

// --- identity / geometry ---------------------------------------------
std::string& Intersection::id() const { return id_; }
node_id_t Intersection::index() const { return index_; }
std::string& Intersection::name() const { return name_; }
std::string& Intersection::zone() const { return zone_; }
bool Intersection::isRoundabout() const { return zone_ == "roundabout"; }
bool Intersection::isWaypoint() const { return zone_ == "waypoint"; }
double Intersection::x() const { return x_; }
double Intersection::y() const { return y_; }
double Intersection::distanceTo(const Intersection& other) const {
    const double dx = x_ - other.x_;
    const double dy = y_ - other.y_;
    return std::sqrt(dx * dx + dy * dy);
}

void Intersection::addOutgoing(road_id_t roadId) {
    if (std::find(outgoing_.begin(), outgoing_.end(), roadId) == outgoing_.end()) {
        outgoing_.push_back(roadId);
    }
}

void Intersection::addIncoming(road_id_t roadId) {
    if (std::find(incoming_.begin(), incoming_.end(), roadId) == incoming_.end()) {
        incoming_.push_back(roadId);
    }
}

void Intersection::removeRoad(road_id_t roadId) {
    outgoing_.erase(std::remove(outgoing_.begin(), outgoing_.end(), roadId),
                    outgoing_.end());
    incoming_.erase(std::remove(incoming_.begin(), incoming_.end(), roadId),
                    incoming_.end());
}

std::vector<road_id_t>& Intersection::outgoing() const { return outgoing_; }
std::vector<road_id_t>& Intersection::incoming() const { return incoming_; }

void Intersection::setSignalGroups(int groups) {
    signalGroups_ = std::clamp(groups, 1, 4);
}

bool Intersection::signalized() const { return signalized_; }
void Intersection::setSignalized(bool on) { signalized_ = on; }
int Intersection::signalGroups() const { return signalGroups_; }
double Intersection::greenDuration() const { return greenDur_; }
double Intersection::amberDuration() const { return amberDur_; }
double Intersection::allRedDuration() const { return allRedDur_; }

void Intersection::setCycle(double green, double amber, double allRed) {
    greenDur_ = std::clamp(green, 5.0, 120.0);
    amberDur_ = std::clamp(amber, 1.0, 10.0);
    allRedDur_ = std::clamp(allRed, 0.0, 10.0);
}

SignalState Intersection::signalStateFor(int group, double simTimeSeconds) const {
    if (!signalized_) return SignalState::Green;
    const double slot = greenDur_ + amberDur_ + allRedDur_;
    const double cycle = slot * signalGroups_;
    double t = std::fmod(simTimeSeconds + phaseOffset_, cycle);
    if (t < 0.0) t += cycle;
    const int active = std::min(signalGroups_ - 1, static_cast<int>(t / slot));
    if (active != group) return SignalState::Red;
    const double u = t - active * slot;
    if (u < greenDur_) return SignalState::Green;
    if (u < greenDur_ + amberDur_) return SignalState::Amber;
    return SignalState::Red;  // clearance interval
}

}  // namespace uts
