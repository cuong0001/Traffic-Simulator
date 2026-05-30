#ifndef VEHICLE_H
#define VEHICLE_H
#include <string>

class vehicle {
protected:
    std::string id;
    double speed;
public:
    Vehicle(std::string id, double speed);
    virtual ~vehicle() = default;
    virtual void move() = 0; // Pure virtual
    virtual std::string get_type() const = 0;
};

#endif
