#ifndef TRAFFICLIGHT_H
#define TRAFFICLIGHT_H
#include "Road.h"

class TrafficLight {
private:
  // Thời lượng mỗi pha (giây). File map lưu dưới dạng số thực (sgreen/samber/sred).
  double yellow, green, red;

  // Số nhóm pha của nút giao (sgroups): 2 = hai hướng luân phiên.
  int groups;

  // 0 = red, 1 = yellow, 2 = green
  unsigned int state;

  double remaining_yellow_time, remaining_red_time, remaining_green_time;

  std::pair<int,int> coordinate;
public:

  TrafficLight();
  // Đèn được nạp từ file map: thời lượng xanh/vàng/đỏ và số nhóm pha.
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

  // Cập nhật trạng thái đèn theo bước thời gian dt (giây)
  void update(double dt);
};

#endif
