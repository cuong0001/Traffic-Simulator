#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "third_party/doctest/doctest.h"

#include "simulation/HistoryStore.hpp"

#include <chrono>
#include <filesystem>
#include <string>

namespace fs = std::filesystem;

namespace {

double real_now() {
    return std::chrono::duration<double>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

// A fresh, empty directory per test so runs never see each other's files.
// Anchored under the same temp_directory_path() convention the rest of the
// suite already uses for scratch map files.
fs::path fresh_dir(const char* name) {
    fs::path p = fs::temp_directory_path() / name;
    std::error_code ec;
    fs::remove_all(p, ec);
    return p;
}

HistoryVehicleRecord make_vehicle(const std::string& id, const std::string& road, float pos) {
    HistoryVehicleRecord v;
    v.id = id;
    v.road = road;
    v.pos = pos;
    v.speed = 10.0f;
    v.lane = 0;
    v.type = "car";
    return v;
}

HistoryFrame make_frame(double real_time, double sim_time) {
    HistoryFrame f;
    f.real_time = real_time;
    f.sim_time = sim_time;
    f.stats.active_vehicles = 0;
    return f;
}

}  // namespace

TEST_CASE("capture and at() round-trip vehicles, lights, events and stats") {
    const fs::path dir = fresh_dir("uts_history_roundtrip");
    HistoryStore store(dir.string(), 86400.0);
    store.load();

    const double t0 = real_now();

    HistoryFrame f = make_frame(t0 - 20.0, 123.5);
    f.vehicles = {make_vehicle("car1", "R1", 5.0f), make_vehicle("car2", "R2", 12.0f)};
    HistoryLightRecord l;
    l.id = "R1";
    l.junction = "J1";
    l.group = 0;
    l.state = 2;
    l.has_head = true;
    l.hx = 1.0f; l.hy = 2.0f; l.hux = 0.5f; l.huy = 0.86f;
    f.lights = {l};
    f.roads = {{"R3", 0.7f, false}};
    HistoryEventRecord e;
    e.id = "ev1"; e.type = "accident"; e.target = "R4"; e.description = "crash"; e.remaining = 30.0;
    f.events = {e};
    f.stats.avg_congestion = 0.42;
    f.stats.active_vehicles = 2;
    f.stats.avg_travel_time = 55.0;
    f.stats.completed_trips = 3;
    f.stats.spawned = 5;
    store.capture(f);

    auto result = store.at(20.0);
    REQUIRE(result.has_value());
    CHECK(result->sim_time == doctest::Approx(123.5));
    REQUIRE(result->vehicles.size() == 2);
    CHECK(result->vehicles[0].id == "car1");
    CHECK(result->vehicles[0].road == "R1");
    CHECK(result->vehicles[0].pos == doctest::Approx(5.0f));
    CHECK(result->vehicles[1].id == "car2");
    REQUIRE(result->lights.size() == 1);
    CHECK(result->lights[0].id == "R1");
    CHECK(result->lights[0].junction == "J1");
    CHECK(result->lights[0].state == 2);
    CHECK(result->lights[0].has_head);
    CHECK(result->lights[0].hx == doctest::Approx(1.0f));
    REQUIRE(result->roads.size() == 1);
    CHECK(result->roads[0].id == "R3");
    CHECK(result->roads[0].congestion == doctest::Approx(0.7f));
    REQUIRE(result->events.size() == 1);
    CHECK(result->events[0].id == "ev1");
    CHECK(result->events[0].description == "crash");
    CHECK(result->stats.completed_trips == 3);
    CHECK(result->stats.spawned == 5);
}

TEST_CASE("a delta frame carries forward unchanged roads and applies changed ones") {
    const fs::path dir = fresh_dir("uts_history_delta");
    HistoryStore store(dir.string(), 86400.0);
    store.load();

    const double t0 = real_now();

    HistoryFrame baseline = make_frame(t0 - 20.0, 1.0);
    baseline.roads = {{"A", 0.5f, false}, {"B", 0.3f, false}};
    store.capture(baseline);

    HistoryFrame next = make_frame(t0 - 10.0, 2.0);
    // Only A changed; B is repeated unchanged (should NOT force a write, but
    // must still resolve when reconstructed); C is new.
    next.roads = {{"A", 0.8f, false}, {"B", 0.3f, false}, {"C", 0.6f, false}};
    store.capture(next);

    auto result = store.at(10.0);
    REQUIRE(result.has_value());
    CHECK(result->sim_time == doctest::Approx(2.0));

    auto find = [&](const std::string& id) -> const HistoryRoadRecord* {
        for (const auto& r : result->roads) if (r.id == id) return &r;
        return nullptr;
    };
    const auto* a = find("A");
    const auto* b = find("B");
    const auto* c = find("C");
    REQUIRE(a != nullptr);
    REQUIRE(b != nullptr);
    REQUIRE(c != nullptr);
    CHECK(a->congestion == doctest::Approx(0.8f));       // changed
    CHECK(b->congestion == doctest::Approx(0.3f));       // carried forward from baseline
    CHECK(c->congestion == doctest::Approx(0.6f));       // newly appeared

    // The baseline itself must still resolve correctly and independently.
    auto baseline_result = store.at(20.0);
    REQUIRE(baseline_result.has_value());
    CHECK(baseline_result->roads.size() == 2);
}

TEST_CASE("a query spanning a segment rotation still resolves the right frame") {
    const fs::path dir = fresh_dir("uts_history_segments");
    HistoryStore store(dir.string(), 86400.0);
    store.load();

    const double t0 = real_now();
    // Segments rotate every real hour (3600s); two captures 4000s apart force
    // a rotation between them.
    HistoryFrame first = make_frame(t0 - 8000.0, 10.0);
    first.vehicles = {make_vehicle("carA", "R1", 1.0f)};
    store.capture(first);

    HistoryFrame second = make_frame(t0 - 4000.0, 20.0);
    second.vehicles = {make_vehicle("carB", "R2", 2.0f)};
    store.capture(second);

    auto r1 = store.at(8000.0);
    REQUIRE(r1.has_value());
    REQUIRE(r1->vehicles.size() == 1);
    CHECK(r1->vehicles[0].id == "carA");

    auto r2 = store.at(4000.0);
    REQUIRE(r2.has_value());
    REQUIRE(r2->vehicles.size() == 1);
    CHECK(r2->vehicles[0].id == "carB");
}

TEST_CASE("eviction removes segments that have fully aged out of the window") {
    const fs::path dir = fresh_dir("uts_history_eviction");
    HistoryStore store(dir.string(), 50.0);  // a short window for a fast test
    store.load();

    const double t0 = real_now();
    // Three captures, each far enough apart (>3600s) to land in its own
    // segment, all anchored to actual "now" so range()'s real-clock-based
    // bookkeeping stays meaningful.
    store.capture(make_frame(t0 - 8000.0, 1.0));
    store.capture(make_frame(t0 - 4000.0, 2.0));
    store.capture(make_frame(t0, 3.0));  // triggers eviction of the first two

    std::size_t bin_files = 0;
    for (const auto& entry : fs::directory_iterator(dir)) {
        if (entry.path().extension() == ".bin") ++bin_files;
    }
    CHECK(bin_files == 1);

    const auto r = store.range();
    REQUIRE_FALSE(r.empty);
    // Only the newest capture (at t0) should remain -- both "ago" values
    // should read as recent, not as far back as the evicted ~8000s frame.
    CHECK(r.oldest_ago < 5.0);
    CHECK(r.newest_ago < 5.0);
}

TEST_CASE("a fresh HistoryStore reloads a previous run's data from disk") {
    const fs::path dir = fresh_dir("uts_history_restart");
    const double t0 = real_now();

    {
        HistoryStore first_run(dir.string(), 86400.0);
        first_run.load();
        HistoryFrame f = make_frame(t0 - 15.0, 42.0);
        f.vehicles = {make_vehicle("survivor", "R9", 3.5f)};
        first_run.capture(f);
    }  // destroyed -- simulates the process exiting

    HistoryStore second_run(dir.string(), 86400.0);
    second_run.load();  // the actual point of the test: rebuild from disk

    auto result = second_run.at(15.0);
    REQUIRE(result.has_value());
    CHECK(result->sim_time == doctest::Approx(42.0));
    REQUIRE(result->vehicles.size() == 1);
    CHECK(result->vehicles[0].id == "survivor");
}

TEST_CASE("sim_time going backward across a map reload does not corrupt real-time ordering") {
    const fs::path dir = fresh_dir("uts_history_reset");
    HistoryStore store(dir.string(), 86400.0);
    store.load();

    const double t0 = real_now();
    // sim_time climbs, then drops (a load() reset), then climbs again -- but
    // real_time only ever goes forward, which is what at() must key off.
    store.capture(make_frame(t0 - 20.0, 100.0));
    store.capture(make_frame(t0 - 10.0, 5.0));
    store.capture(make_frame(t0, 8.0));

    auto oldest = store.at(20.0);
    auto middle = store.at(10.0);
    auto newest = store.at(0.0);
    REQUIRE(oldest.has_value());
    REQUIRE(middle.has_value());
    REQUIRE(newest.has_value());
    CHECK(oldest->sim_time == doctest::Approx(100.0));
    CHECK(middle->sim_time == doctest::Approx(5.0));
    CHECK(newest->sim_time == doctest::Approx(8.0));
}

TEST_CASE("at() clamps to the retained range instead of erroring at the edges") {
    const fs::path dir = fresh_dir("uts_history_clamp");
    HistoryStore store(dir.string(), 86400.0);
    store.load();

    const double t0 = real_now();
    store.capture(make_frame(t0 - 30.0, 1.0));
    store.capture(make_frame(t0, 2.0));

    auto newest = store.at(0.0);
    REQUIRE(newest.has_value());
    CHECK(newest->sim_time == doctest::Approx(2.0));

    // Absurdly far back: must clamp to the oldest retained frame, not error.
    auto beyond = store.at(1e9);
    REQUIRE(beyond.has_value());
    CHECK(beyond->sim_time == doctest::Approx(1.0));
}

TEST_CASE("to_state_json shapes a frame the way the frontend's applyState() expects") {
    HistoryFrame f = make_frame(5.0, 5.0);
    f.vehicles = {make_vehicle("car1", "R1", 5.0f)};
    HistoryLightRecord l;
    l.id = "R1"; l.junction = "J1"; l.group = 0; l.state = 1;
    f.lights = {l};
    f.roads = {{"R1", 0.5f, true}};
    f.stats.active_vehicles = 1;

    const std::string json = HistoryStore::to_state_json(f);
    CHECK(json.find("\"vehicles\":[{\"id\":\"car1\"") != std::string::npos);
    CHECK(json.find("\"lights\":[{\"id\":\"R1\"") != std::string::npos);
    CHECK(json.find("\"roads\":[{\"id\":\"R1\",\"congestion\":0.5,\"closed\":true") != std::string::npos);
    CHECK(json.find("\"stats\":{") != std::string::npos);
}
