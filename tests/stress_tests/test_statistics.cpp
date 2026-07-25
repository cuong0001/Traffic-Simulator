#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "third_party/doctest/doctest.h"
#include "utils/StatisticsManager.hpp"

TEST_SUITE("StatisticsManager - Stress Tests") {

    TEST_CASE("Handle 10 million completed trips without overflow") {
        StatisticsManager stats;
        
        // Giả lập 10 triệu chuyến xe hoàn thành trong một kịch bản giao thông lớn
        for(int i = 0; i < 10000000; ++i) {
            stats.recordTripCompleted(300.0, 5000.0, "car");
        }
        
        CHECK(stats.completedTrips() == 10000000);
        // Kiểm tra xem biến double có bị sai số khi cộng dồn hàng triệu lần không
        CHECK(stats.averageTravelTime() == doctest::Approx(300.0));
        CHECK(stats.averageRouteLength() == doctest::Approx(5000.0));
    }

    TEST_CASE("Rapid history insertion maintains kMaxSamples efficiently") {
        StatisticsManager stats;
        
        // Đẩy 1 triệu snapshot lịch sử liên tục
        for(int i = 0; i < 1000000; ++i) {
            stats.pushSample(static_cast<double>(i), 0.75, 100);
        }
        
        // Đảm bảo sau 1 triệu lần chèn và xóa, kích thước vẫn luôn chỉ đúng 240
        REQUIRE(stats.history().size() == 240);
        // Phần tử cuối cùng phải là lần lặp thứ 999999
        CHECK(stats.history().back().simTime == doctest::Approx(999999.0));
    }
}