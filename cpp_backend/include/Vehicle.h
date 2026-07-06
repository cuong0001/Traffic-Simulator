#ifndef VEHICLE_H
#define VEHICLE_H

class Vehicle {
private:
// Các thuộc tính bắt buộc
  std::pair<int,int> coordinate;// tọa độ xe
  std::pair<int,int> destination;// đích đến của xe
  std::pair<int,int> size; // kích thước
  std::pair<int,int> vector_a; // gia tốc xe
  std::pair<int,int> vector_v; // vận tốc xe
// Các thuộc tính thêm
  unsigned int weight;
public:
  virtual Vehicle() = 0;
  // tạo ra hàng loạt getter, không cho setter vào để tránh lập trình viên thay đổi bbiến trong quá trình xe di chuyển.
  std::pair<int,int> get_coordinate();
  std::pair<int,int> get_size();
  std::pair<int,int> get_velocity();
  std::pair<int,int> get_acceleration();
  unsigned int get_weight();

}


#endif
