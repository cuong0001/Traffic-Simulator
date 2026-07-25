#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "third_party/doctest/doctest.h"
#include "utils/StatisticsManager.hpp"
#include <string>

TEST_SUITE("StatisticsManager - Edge Cases") {
    
    TEST_CASE("Safe division when no data exists") {
        StatisticsManager stats;
        // Kiểm tra việc gọi các hàm trung bình khi chưa có dữ liệu có bị lỗi Crash (chia cho 0) không.
        CHECK(stats.averageTravelTime() == 0.0);
        CHECK(stats.averageRouteLength() == 0.0);
        
        StatisticsManager::AlgorithmStats empty_algo;
        CHECK(empty_algo.avgMicros() == 0.0);
        CHECK(empty_algo.avgNodes() == 0.0);
    }

    TEST_CASE("Handling empty strings for map keys") {
        StatisticsManager stats;
        // Gửi loại xe và thuật toán là chuỗi rỗng ""
        stats.recordTripCompleted(10.0, 100.0, "");
        stats.recordRouteComputation("", 5.0, 10, true);
        
        CHECK(stats.completedTrips() == 1);
        auto algo_stats = stats.algorithmStats();
        
        // Đảm bảo std::map vẫn lưu trữ và không gặp lỗi với chuỗi rỗng
        REQUIRE(algo_stats.find("") != algo_stats.end());
        CHECK(algo_stats.at("").computations == 1);
        
        std::string jsonStr = stats.toJson();
        CHECK(jsonStr.find("\"\":1") != std::string::npos);
    }

    TEST_CASE("Handling negative values safely") {
        StatisticsManager stats;
        // Mô phỏng lỗi logic từ đầu vào hệ thống (truyền vào số âm)
        stats.recordTripCompleted(-50.0, -1000.0, "car");
        
        CHECK(stats.totalTravelTime() == -50.0);
        CHECK(stats.averageRouteLength() == doctest::Approx(-1000.0));
    }
}