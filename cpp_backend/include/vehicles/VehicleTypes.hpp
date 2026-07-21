#pragma once

#include <memory>
#include <random>

#include "vehicles/Vehicle.hpp"

namespace uts {

/// Regular passenger car: baseline behaviour.
class Car : public Vehicle {
public:
    using Vehicle::Vehicle;
    std::string type() const override { return "car"; }
};

/// Public bus: slower, fully bound by traffic rules.
class Bus : public Vehicle {
public:
    using Vehicle::Vehicle;
    std::string type() const override { return "bus"; }
    double speedFactor() const override { return 0.8; }
    double lengthMeters() const override { return 6.5; }
    /// On 3+ lane roads buses keep out of the innermost (fast) lane.
    std::vector<int> allowedLanes(int totalLanes) const override {
        if (totalLanes < 3) return Vehicle::allowedLanes(totalLanes);
        std::vector<int> lanes;
        for (int i = 1; i < totalLanes; ++i) lanes.push_back(i);
        return lanes;
    }
};

/// Emergency vehicle: fast, weaves through congestion, ignores red lights.
class EmergencyVehicle : public Vehicle {
public:
    using Vehicle::Vehicle;
    std::string type() const override { return "emergency"; }
    double speedFactor() const override { return 1.35; }
    double congestionSensitivity() const override { return 0.25; }
    bool ignoresTrafficLights() const override { return true; }
    double lengthMeters() const override { return 5.0; }
    /// MOBIL politeness 0: sirens don't negotiate.
    double lanePoliteness() const override { return 0.0; }
};

/// Motorbike (optional extra): nimble in congestion, slightly slower top speed.
class Motorbike : public Vehicle {
public:
    using Vehicle::Vehicle;
    std::string type() const override { return "motorbike"; }
    double speedFactor() const override { return 0.95; }
    double congestionSensitivity() const override { return 0.6; }
    double lengthMeters() const override { return 2.0; }
    double widthMeters() const override { return 0.8; }
    /// On 3+ lane roads bikes stay in the two innermost lanes.
    std::vector<int> allowedLanes(int totalLanes) const override {
        if (totalLanes < 3) return Vehicle::allowedLanes(totalLanes);
        return {0, 1};
    }
};

/**
 * Factory design pattern: centralises the creation of vehicles so the
 * simulator never news up concrete classes directly.
 */
class VehicleFactory {
public:
    static std::unique_ptr<Vehicle> create(const std::string& type,
                                           const std::string& id,
                                           const std::string& originId,
                                           const std::string& destinationId,
                                           std::shared_ptr<PathfindingStrategy> strategy,
                                           CostMode mode);

    /// Weighted random type: mostly cars, some buses/bikes, rare emergencies.
    static std::string randomType(std::mt19937& rng);
};

}  // namespace uts
