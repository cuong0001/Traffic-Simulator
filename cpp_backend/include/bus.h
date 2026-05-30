#ifndef BUS_H
#define BUS_H
#include "vehicle.h"
class bus : public vehicle {
public:
    bus(std::string id);
    void move() override;
    std::string get_type() const override { return "Bus"; }
};

#endif
