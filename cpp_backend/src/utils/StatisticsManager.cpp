#include "utils/StatisticsManager.hpp"
#include "utils/Json.hpp"
#include <sstream>

void StatisticsManager::recordTripCompleted(double travelTimeSeconds,
                                            double routeLengthMeters,
                                            const std::string& vehicleType) 
{
    ++completedTrips_;
    totalTravelTime_ += travelTimeSeconds;
    totalRouteLength_ += routeLengthMeters;
    ++tripsByType_[vehicleType];
}

int StatisticsManager::completedTrips() const 
{ 
    return completedTrips_; 
}

double StatisticsManager::averageTravelTime() const 
{
    return completedTrips_ ? totalTravelTime_ / completedTrips_ : 0.0;
}

double StatisticsManager::averageRouteLength() const 
{
    return completedTrips_ ? totalRouteLength_ / completedTrips_ : 0.0;
}

double StatisticsManager::totalTravelTime() const 
{
    return totalTravelTime_; 
}

void StatisticsManager::recordRouteComputation(const std::string& algorithm,
                                               double micros,
                                               std::size_t nodesExplored,
                                               bool found) 
{
    ++routeComputations_;
    AlgorithmStats& s = algoStats_[algorithm];
    ++s.computations;
    if (!found) ++s.failures;
    s.totalMicros += micros;
    s.totalNodesExplored += static_cast<double>(nodesExplored);
}

const std::map<std::string, StatisticsManager::AlgorithmStats>& StatisticsManager::algorithmStats() const 
{
    return algoStats_;
}

int StatisticsManager::totalRecalculations() const 
{
    return routeComputations_;
}

void StatisticsManager::pushSample(double simTime, double avgCongestion,
                                   int activeVehicles) 
{
    history_.push_back({simTime, avgCongestion, activeVehicles, averageTravelTime()});
    while (history_.size() > kMaxSamples) history_.pop_front();
}

const std::deque<StatisticsManager::Sample>& StatisticsManager::history() const 
{
    return history_;
}

std::string StatisticsManager::toJson(bool includeHistory) const 
{
    std::ostringstream os;
    os << "{";
    os << "\"completedTrips\":" << completedTrips_ << ",";
    os << "\"avgTravelTime\":" << json::num(averageTravelTime(), 2) << ",";
    os << "\"avgRouteLength\":" << json::num(averageRouteLength(), 2) << ",";
    os << "\"routeComputations\":" << routeComputations_ << ",";

    os << "\"tripsByType\":{";
    bool first = true;
    for (const auto& [type, count] : tripsByType_) {
        if (!first) os << ",";
        first = false;
        os << json::str(type) << ":" << count;
    }
    os << "},";

    os << "\"algorithms\":{";
    first = true;
    for (const auto& [name, s] : algoStats_) {
        if (!first) os << ",";
        first = false;
        os << json::str(name) << ":{"
           << "\"computations\":" << s.computations << ","
           << "\"failures\":" << s.failures << ","
           << "\"avgMicros\":" << json::num(s.avgMicros(), 2) << ","
           << "\"avgNodesExplored\":" << json::num(s.avgNodes(), 1) << "}";
    }
    os << "},";

    os << "\"history\":[";
    if (includeHistory) {
        first = true;
        for (const Sample& s : history_) {
            if (!first) os << ",";
            first = false;
            os << "{\"t\":" << json::num(s.simTime, 1)
               << ",\"congestion\":" << json::num(s.avgCongestion, 3)
               << ",\"vehicles\":" << s.activeVehicles
               << ",\"avgTravel\":" << json::num(s.avgTravelTime, 1) << "}";
        }
    }
    os << "]}";
    return os.str();
}