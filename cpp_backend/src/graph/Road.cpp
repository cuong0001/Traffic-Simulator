#include "../../include/Road.h"
#include "../../include/Node.h"
#include "../../include/Vehicle.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

Road::Road(std::string id, Node* source, Node* target, int lanes, double speed_limit,
           int capacity, bool is_one_way, double length, Geometry* geometry)
    : id(std::move(id)), source(source), target(target),
      lanes(lanes), speed_limit(speed_limit),
      capacity(capacity), is_one_way(is_one_way),
      is_internal(false), congestion_level(0.0), geometry(geometry) {

    // Length comes from the map file when given; otherwise measure it from the
    // endpoints.
    if (length > 0.0) {
        this->length = length;
    } else {
        double dx = target->x - source->x;
        double dy = target->y - source->y;
        this->length = std::sqrt(dx * dx + dy * dy);
    }

    // base_weight = cost of traversing the whole road under ideal conditions
    base_weight = this->length;
    travel_cost = base_weight;

    // Thêm cạnh vào danh sách kề của 2 Node để phục vụ duyệt đồ thị (tìm đường)
    source->outgoing_edges.push_back(this);
    target->incoming_edges.push_back(this);
}

void Road::add_vehicle(Vehicle* vehicle) {
    vehicles.push_back(vehicle);
}

void Road::remove_vehicle(Vehicle* vehicle) {
    vehicles.erase(std::remove(vehicles.begin(), vehicles.end(), vehicle), vehicles.end());
}

double Road::get_congestion_level() const {
    return congestion_level;
}

double Road::get_weight() const {
    return travel_cost;
}

// Cập nhật chi phí di chuyển theo mức độ tắc nghẽn hiện tại
void Road::update() {
    travel_cost = base_weight * (1.0 + congestion_level);
}

// Tìm xe gần nhất phía trước current_vehicle trên đường này (xe có
// position_on_road nhỏ nhất trong số các xe nằm trước current_vehicle).
// Trả về (xe phía trước, khoảng cách tới xe đó).
// Nếu phía trước thông thoáng: trả về (nullptr, +infinity).
// Nếu current_vehicle là nullptr (xe chuẩn bị đi vào đường này): tính từ đầu đường.
std::pair<Vehicle*, double> Road::get_lead_vehicle(Vehicle* current_vehicle) {
    double ref_position = current_vehicle ? current_vehicle->position_on_road : 0.0;

    Vehicle* lead = nullptr;
    double min_gap = std::numeric_limits<double>::infinity();

    for (Vehicle* other : vehicles) {
        if (!other || other == current_vehicle) continue;

        double gap = other->position_on_road - ref_position;
        if (gap > 0.0 && gap < min_gap) {
            min_gap = gap;
            lead = other;
        }
    }

    return {lead, min_gap};
}
