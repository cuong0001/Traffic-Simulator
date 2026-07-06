#ifndef TRAFFICLIGHT_H
#define TRAFFICLIGHT_H
#include "Road.h" 

class TrafficLight {
private:
  unsigned int yellow, green, red;

  // 0 = red, 1 = yellow, 2 = green
  unsigned int state;

  unsigned int remaining_yellow_time, remaining_red_time, remaining_green_time;

  std::pair<int,int> coordinate;
public:

  TrafficLight();
  unsigned int get_yellow();
  unsigned int get_green();
  unsigned int get_red();
  
  unsigned int set_remaining_yellow();
  unsigned int set_remaining_green();
  unsigned int set_remaining_red();
  
  void create_obstacle_on_red();

  // Cập nhật trạng thái đèn theo bước thời gian dt (giây)
  void update(double dt);
};

#endif
