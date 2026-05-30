#ifndef EMERGENCY_VEHICLE_H
#define EMERGENCY_VEHICLE_H
#include "vehicle.h"
class emergency_vehicle : public vehicle {
public:
    emergency_vehicle(std::string id);
    void move() override;
    std::string get_type() const override { return "Emergency"; }
};
#endif
