#include "vehicles/VehicleTypes.hpp"

std::unique_ptr<Vehicle> VehicleFactory::create(
    const std::string& type, 
    const std::string& id, 
    double max_speed, 
    double max_accel, 
    double max_decel) 
{
    if (type == "bus") {
        return std::make_unique<Bus>(id, max_speed, max_accel, max_decel);
    }
    if (type == "emergency") {
        return std::make_unique<EmergencyVehicle>(id, max_speed, max_accel, max_decel);
    }
    if (type == "motorbike") {
        return std::make_unique<Motorbike>(id, max_speed, max_accel, max_decel);
    }
    // Default, and "car": an ordinary passenger car.
    return std::make_unique<Car>(id, max_speed, max_accel, max_decel);
}

std::string VehicleFactory::random_type(std::mt19937& rng) {
    std::uniform_int_distribution<int> dist(0, 99);
    const int roll = dist(rng);
    if (roll < 68) return "car";
    if (roll < 84) return "motorbike";
    if (roll < 96) return "bus";
    return "emergency";
}
