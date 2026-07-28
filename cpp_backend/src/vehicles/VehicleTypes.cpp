#pragma once
/**
 * vehicletypes.hpp
 * ----------------
 * Concrete vehicle classes (inheritance + polymorphism) and the
 * VehicleFactory (Factory design pattern) used to spawn them.
 */

#include <memory>
#include <random>
#include <string>
#include <vector>

#include "vehicles/Vehicle.hpp"

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

/// Motorbike: nimble in congestion, slightly slower top speed.
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
 * Factory design pattern: centralises the creation of vehicles.
 */
class VehicleFactory {
public:
    static std::unique_ptr<Vehicle> create(const std::string& type,
                                           const std::string& id,
                                           double max_speed = 60.0,
                                           double max_accel = 3.0,
                                           double max_decel = 4.5);

    /// Weighted random type: mostly cars, some buses/bikes, rare emergencies.
    static std::string randomType(std::mt19937& rng);
};
