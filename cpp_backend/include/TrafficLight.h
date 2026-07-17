#ifndef TRAFFICLIGHT_H
#define TRAFFICLIGHT_H
#include "Road.h"

class TrafficLight {
private:
  // Duration of each phase (seconds). The map stores these as reals (sgreen/samber/sred).
  double yellow, green, red;

  // Number of signal groups at the junction (sgroups): 2 = two directions alternating.
  int groups;

  // 0 = red, 1 = yellow, 2 = green
  unsigned int state;

  double remaining_yellow_time, remaining_red_time, remaining_green_time;

  std::pair<int,int> coordinate;
public:

  TrafficLight();
  // Light loaded from the map file: green/yellow/red durations and group count.
  TrafficLight(double green, double yellow, double red, int groups = 2);

  double get_green_duration() const;
  double get_yellow_duration() const;
  double get_red_duration() const;
  int get_groups() const;
  unsigned int get_state() const;

  double get_remaining_yellow();
  double get_remaining_green();
  double get_remaining_red();

  double set_remaining_yellow();
  double set_remaining_green();
  double set_remaining_red();

  void create_obstacle_on_red();

  // Advance the light state by a time step dt (seconds)
  void update(double dt);
};

#endif
