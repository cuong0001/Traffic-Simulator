#ifndef TRAFFICSIMULATOR_H
#define TRAFFICSIMULATOR_H
#include "Road.h"
#include "Graph.h"
#include "TrafficLight.h"
#include "Vehicle.h"
#include "Node.h"
#include "building.h"

class TrafficSimulator {
private:
public:
  void change_simulation_speed();
  void next_frame();
  void spawn_vehicle();
  void add_building();
  void add_ambulance();
  void delete_road();
  void isolate_road(unsigned int timer);
  void delete_vehicle();
// Các hàm event
  void vechicle_shutdown(Vehicle*);
  void death_related(Building*);
  
};

#endif
