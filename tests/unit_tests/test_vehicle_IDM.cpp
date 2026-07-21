#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "vehicles/Vehicle.hpp"
#include "vehicles/VehicleTypes.hpp"

namespace uts {

// Mock Strategy phục vụ testing
class MockStrategy : public PathfindingStrategy {
public:
    std::string name() const override { return "Mock"; }
    RouteResult findRoute(const Graph&, node_id_t, node_id_t, CostMode) override {
        return RouteResult{};
    }
};

void testPolymorphism() {
    auto strat = std::make_shared<MockStrategy>();

    Car car("V1", "A", "B", strat, CostMode::Time);
    assert(car.type() == "car");
    assert(std::abs(car.speedFactor() - 1.0) < 1e-5);
    assert(std::abs(car.congestionSensitivity() - 1.0) < 1e-5);
    assert(car.ignoresTrafficLights() == false);
    assert(std::abs(car.lengthMeters() - 4.5) < 1e-5);

    Bus bus("V2", "A", "B", strat, CostMode::Time);
    assert(bus.type() == "bus");
    assert(std::abs(bus.speedFactor() - 0.8) < 1e-5);
    assert(std::abs(bus.lengthMeters() - 6.5) < 1e-5);
    auto busLanes = bus.allowedLanes(4);
    std::vector<int> expectedBusLanes = {1, 2, 3};
    assert(busLanes == expectedBusLanes);

    EmergencyVehicle emg("V3", "A", "B", strat, CostMode::Time);
    assert(emg.type() == "emergency");
    assert(emg.ignoresTrafficLights() == true);
    assert(std::abs(emg.lanePoliteness() - 0.0) < 1e-5);

    Motorbike bike("V4", "A", "B", strat, CostMode::Time);
    assert(bike.type() == "motorbike");
    assert(std::abs(bike.widthMeters() - 0.8) < 1e-5);
    auto bikeLanes = bike.allowedLanes(4);
    std::vector<int> expectedBikeLanes = {0, 1};
    assert(bikeLanes == expectedBikeLanes);

    std::cout << "  [PASSED] Polymorphism Tests\n";
}

void testVehicleFactory() {
    auto strat = std::make_shared<MockStrategy>();

    auto v1 = VehicleFactory::create("car", "1", "A", "B", strat, CostMode::Time);
    assert(v1->type() == "car");

    auto v2 = VehicleFactory::create("bus", "2", "A", "B", strat, CostMode::Time);
    assert(v2->type() == "bus");

    auto v3 = VehicleFactory::create("emergency", "3", "A", "B", strat, CostMode::Time);
    assert(v3->type() == "emergency");

    auto v4 = VehicleFactory::create("unknown", "4", "A", "B", strat, CostMode::Time);
    assert(v4->type() == "car"); // Mặc định về car

    std::cout << "  [PASSED] VehicleFactory Tests\n";
}

void testIDMMath() {
    // Tăng tốc tự do
    double a1 = Vehicle::idmAccel(0.0, 10.0, 0.0, 1000.0);
    assert(std::abs(a1 - 2.5) < 1e-2);

    // Dừng lại khi gặp vật cản
    double a2 = Vehicle::idmAccel(0.0, 10.0, 0.0, 2.0);
    assert(std::abs(a2 - 0.0) < 1e-2);

    // Phanh gấp khi quá gần
    double a3 = Vehicle::idmAccel(10.0, 10.0, 0.0, 1.0);
    assert(a3 < -2.0);

    std::cout << "  [PASSED] IDM Acceleration Math Tests\n";
}

} // namespace uts

int main() {
    std::cout << "=== RUNNING UNIT TESTS (No 3rd-party libs) ===\n";
    uts::testPolymorphism();
    uts::testVehicleFactory();
    uts::testIDMMath();
    std::cout << ">>> ALL UNIT TESTS PASSED SUCCESSFULLY! <<<\n\n";
    return 0;
}
