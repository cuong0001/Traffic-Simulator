#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>

#include "vehicles/Vehicle.hpp"
#include "vehicles/VehicleTypes.hpp"

namespace uts {

class DummyStrategy : public PathfindingStrategy {
public:
    std::string name() const override { return "Dummy"; }
    RouteResult findRoute(const Graph&, node_id_t, node_id_t, CostMode) override {
        return RouteResult{};
    }
};

void testIDMZeroAndExtremeInputs() {
    // Gap siêu nhỏ -> Không bị NaN
    double a1 = Vehicle::idmAccel(10.0, 10.0, 0.0, 0.000001);
    assert(!std::isnan(a1));
    assert(std::isinf(a1) || a1 < -1e5);

    // Tốc độ mong muốn v0 = 0 -> Không văng lỗi chia cho 0
    double a2 = Vehicle::idmAccel(5.0, 0.0, 0.0, 10.0);
    assert(!std::isnan(a2));

    std::cout << "  [PASSED] IDM Zero/Extreme Inputs Edge Test\n";
}

void testSpecialCharacterIDs() {
    auto strat = std::make_shared<DummyStrategy>();

    // ID rỗng
    Car c1("", "", "", strat, CostMode::Time);
    assert(c1.id() == "");

    // ID ký tự lạ / Unicode
    Car c2("Xe_MÁY_#999!", "StartNode", "EndNode", strat, CostMode::Time);
    assert(c2.id() == "Xe_MÁY_#999!");

    std::cout << "  [PASSED] Special Character IDs Edge Test\n";
}

void testNullStrategyHandling() {
    auto strat1 = std::make_shared<DummyStrategy>();

    Car car("V_Edge", "A", "B", strat1, CostMode::Time);

    // Truyền nullptr vào setStrategy (không crash, giữ lại strategy cũ)
    car.setStrategy(nullptr);
    assert(&car.strategy() == strat1.get());

    std::cout << "  [PASSED] Null Strategy Edge Test\n";
}

void testZoneWaitCapThreshold() {
    Car car("V_Wait", "A", "B", std::make_shared<DummyStrategy>(), CostMode::Time);

    car.addZoneWait(10.0);
    assert(std::abs(car.zoneWait() - 10.0) < 1e-5);

    // Cộng tiếp 10s, ngưỡng tối đa khống chế là 15.0s
    car.addZoneWait(10.0);
    assert(std::abs(car.zoneWait() - 15.0) < 1e-5);

    std::cout << "  [PASSED] Zone Wait Cap Edge Test\n";
}

} // namespace uts

int main() {
    std::cout << "=== RUNNING EDGE TESTS (No 3rd-party libs) ===\n";
    uts::testIDMZeroAndExtremeInputs();
    uts::testSpecialCharacterIDs();
    uts::testNullStrategyHandling();
    uts::testZoneWaitCapThreshold();
    std::cout << ">>> ALL EDGE TESTS PASSED SUCCESSFULLY! <<<\n\n";
    return 0;
}
