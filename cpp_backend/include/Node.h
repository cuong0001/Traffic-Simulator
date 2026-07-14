#pragma once
#include <string>
#include <vector>
#include "TrafficLight.h" 
class Road;

class Node {
public:
    std::string name;         // Định danh (id) của node, dùng làm khóa trong TrafficGraph
    std::string display_name; // Tên hiển thị, ví dụ "Hillside & 1st"
    std::string zone;         // Khu vực chứa node ("", "downtown", "roundabout", ...)
    double x;
    double y;
    bool is_building;
    bool is_roundabout;
    bool is_signalized;       // Node có đèn giao thông hay không
    double radius;
    std::vector<Road*> outgoing_edges;
    std::vector<Road*> incoming_edges;

    TrafficLight* light_system;

    Node(std::string name, double x, double y);
    ~Node(); 

    bool is_intersection() const;
    void update(double dt); 
}; 
