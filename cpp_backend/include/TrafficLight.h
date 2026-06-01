#ifndef TRAFFICLIGHT_H
#define TRAFFICLIGHT_H
#include "Road.h"

class TrafficLight {
private:
  unsigned int yellow, green, red;
  std::pair<int,int> coordinate;
  Road *
public:
  TrafficLight();
  unsigned int get_yellow();
  unsigned int get_green();
  unsigned int get_red();

  unsigned int set_yellow();
  unsigned int set_red);
  unsigned int set_red();
  
  void create_obstacle_on_red();
}
