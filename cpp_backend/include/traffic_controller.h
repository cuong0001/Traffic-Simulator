#ifndef TRAFFIC_CONTROLLER_H
#define TRAFFIC_CONTROLLER_H
#include "traffic_simulator.h"

class traffic_controller {
private:
    traffic_simulator* simulator;
public:
    traffic_controller(traffic_simulator* sim);
    void process_inputs();
};

#endif
