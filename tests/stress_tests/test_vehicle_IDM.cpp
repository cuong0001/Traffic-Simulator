#include <chrono>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <vector>

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

void testMassCreationPerformance() {
    constexpr int kCount = 100000;
    std::mt19937 rng(42);
    auto strat = std::make_shared<DummyStrategy>();

    std::vector<std::unique_ptr<Vehicle>> vehicles;
    vehicles.reserve(kCount);

    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < kCount; ++i) {
        std::string t = VehicleFactory::randomType(rng);
        vehicles.push_back(VehicleFactory::create(t, "V_" + std::to_string(i), "A", "B", strat, CostMode::Time));
    }

    auto end = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(end - start).count();

    std::cout << "  - Created " << kCount << " vehicles in " << ms << " ms.\n";
    if (ms > 2000.0) {
        std::cerr << "  [FAIL] Mass creation took too long!\n";
        exit(1);
    }
    std::cout << "  [PASSED] Mass Creation Performance Test\n";
}

void testIDMComputationStress() {
    constexpr int kLoop = 10000000;
    double sum = 0.0;

    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < kLoop; ++i) {
        double v = static_cast<double>(i % 30);
        double v0 = 13.88;
        double vLead = static_cast<double>((i + 5) % 30);
        double gap = static_cast<double>((i % 50) + 1);
        sum += Vehicle::idmAccel(v, v0, vLead, gap);
    }

    auto end = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(end - start).count();

    std::cout << "  - Computed 10M IDM steps (checksum: " << sum << ") in " << ms << " ms.\n";
    if (ms > 1000.0) {
        std::cerr << "  [FAIL] IDM Computation took too long!\n";
        exit(1);
    }
    std::cout << "  [PASSED] IDM Computation Stress Test\n";
}

} // namespace uts

int main() {
    std::cout << "=== RUNNING STRESS TESTS (No 3rd-party libs) ===\n";
    uts::testMassCreationPerformance();
    uts::testIDMComputationStress();
    std::cout << ">>> ALL STRESS TESTS PASSED SUCCESSFULLY! <<<\n\n";
    return 0;
}
