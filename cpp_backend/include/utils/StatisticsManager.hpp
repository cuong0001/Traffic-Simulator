/*
 * Collects travel metrics, algorithm metrics and traffic metrics
 * (Statistics and Analysis Module of the project brief).
*/
#pragma once

#include <deque>
#include <map>
#include <string>

class StatisticsManager {
public:
    // Travel metrics
    void recordTripCompleted(double travelTimeSeconds, double routeLengthMeters,
                             const std::string& vehicleType); //Aggregate time, distance, and vehicle classification when a vehicle completes a trip.

    int completedTrips() const; //Total number of completed trips.
    double averageTravelTime() const; //Average travel time of completed trips in seconds.
    double averageRouteLength() const; // Average route length per completed trip, in meters.
    double totalTravelTime() const; //Total travel time of all completed trips in seconds.

    // Algorithm metrics
    struct AlgorithmStats //Stores statistics for a specific pathfinding algorithm.
    {
        int computations = 0;
        int failures = 0;
        double totalMicros = 0.0;
        double totalNodesExplored = 0.0;
        double avgMicros() const {
            return computations ? totalMicros / computations : 0.0;
        }
        double avgNodes() const {
            return computations ? totalNodesExplored / computations : 0.0;
        }
    };

    void recordRouteComputation(const std::string& algorithm, double micros,
                                std::size_t nodesExplored, bool found); //record the performance of pathfinding algorithms
    const std::map<std::string, AlgorithmStats>& algorithmStats() const;
        
    int totalRecalculations() const; //the total number of times the entire simulation has to calculate or recalculate the route.

    // Traffic metrics
    struct Sample //Stores a snapshot of traffic metrics at a specific simulation time.
    {
        double simTime;
        double avgCongestion;
        int activeVehicles;
        double avgTravelTime;
    };
    void pushSample(double simTime, double avgCongestion, int activeVehicles);
    
    const std::deque<Sample>& history() const;

    std::string toJson(bool includeHistory = true) const;

private:
    int completedTrips_ = 0;
    double totalTravelTime_ = 0.0;
    double totalRouteLength_ = 0.0;
    std::map<std::string, int> tripsByType_;

    int routeComputations_ = 0;
    std::map<std::string, AlgorithmStats> algoStats_;

    std::deque<Sample> history_;
    static constexpr std::size_t kMaxSamples = 240;
};