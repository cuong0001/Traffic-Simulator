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
class Car final : public Vehicle {
public:
    using Vehicle::Vehicle;
    std::string type() const override { return "car"; }
};

/// Public bus: slower, fully bound by traffic rules.
class Bus final : public Vehicle {
public:
    using Vehicle::Vehicle;
    std::string type() const override { return "bus"; }
    double speed_factor() const override { return 0.8; }
    double length_meters() const override { return 6.5; }

    /// On 3+ lane roads buses keep out of the innermost (fast) lane.
    std::vector<int> allowed_lanes(int total_lanes) const override {
        if (total_lanes < 3) return Vehicle::allowed_lanes(total_lanes);
        std::vector<int> lanes;
        for (int i = 1; i < total_lanes; ++i) lanes.push_back(i);
        return lanes;
    }
};

/// Emergency vehicle: fast, weaves through congestion, ignores red lights.
class EmergencyVehicle final : public Vehicle {
public:
    using Vehicle::Vehicle;
    std::string type() const override { return "emergency"; }
    double speed_factor() const override { return 1.35; }
    double congestion_sensitivity() const override { return 0.25; }
    bool ignores_lights() const override { return true; }
    double length_meters() const override { return 5.0; }

    /// MOBIL politeness 0: sirens don't negotiate.
    double lane_politeness() const override { return 0.0; }
};

/// Motorbike: nimble in congestion, slightly slower top speed.
class Motorbike final : public Vehicle {
public:
    using Vehicle::Vehicle;
    std::string type() const override { return "motorbike"; }
    double speed_factor() const override { return 0.95; }
    double congestion_sensitivity() const override { return 0.6; }
    double length_meters() const override { return 2.0; }
    double width_meters() const override { return 0.8; }

    /// On 3+ lane roads bikes stay in the two innermost lanes.
    std::vector<int> allowed_lanes(int total_lanes) const override {
        if (total_lanes < 3) return Vehicle::allowed_lanes(total_lanes);
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
                                           double max_speed = 16.7,
                                           double max_accel = 3.0,
                                           double max_decel = 4.5);

    /// Weighted random type: mostly cars, some buses/bikes, rare emergencies.
    static std::string random_type(std::mt19937& rng);

    /// Maps an arbitrary caller-supplied type string to one of the four
    /// concrete classes create() actually builds ("bus"/"emergency"/
    /// "motorbike", else "car" -- the same fallback create() uses). Callers
    /// that need to build a vehicle's id BEFORE construction (spawn_locked)
    /// use this so the id's type prefix always matches what type() reports
    /// afterward, instead of echoing back whatever string the caller passed.
    static std::string canonical_type(const std::string& type);
};
