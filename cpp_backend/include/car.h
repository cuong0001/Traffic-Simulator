#ifndef CAR_H
#define CAR_H
#include "vehicle.h"
class car : public vehicle {
public:
    car(std::string id);
    void move() override;
    std::string get_type() const override { return "Car"; }
};

#endif
