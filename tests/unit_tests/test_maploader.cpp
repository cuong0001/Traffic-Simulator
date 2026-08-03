#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "third_party/doctest/doctest.h"

#include <filesystem>
#include <string>
#include <system_error>

#include "core/Building.hpp"
#include "core/Graph.hpp"
#include "core/Intersection.hpp"
#include "core/Road.hpp"
#include "core/TrafficLight.hpp"
#include "core/Zone.hpp"
#include "utils/MapLoader.hpp"

namespace {

// Three nodes, one one-way and one two-way street, one anchored building and
// one background zone: every feature kind write_geojson has to emit.
const char* kMap = R"JSON({
  "type": "FeatureCollection",
  "features": [
    {"type":"Feature",
     "properties":{"uts:kind":"node","id":"A","name":"Alpha & 1st","zone":"downtown"},
     "geometry":{"type":"Point","coordinates":[0,0]}},
    {"type":"Feature",
     "properties":{"uts:kind":"node","id":"B","name":"Beta Junction","zone":"downtown",
                   "signal":true,"sgroups":3,"sgreen":18.5,"samber":4,"sred":2.5},
     "geometry":{"type":"Point","coordinates":[120,0]}},
    {"type":"Feature",
     "properties":{"uts:kind":"node","id":"C","name":"Gamma"},
     "geometry":{"type":"Point","coordinates":[120,90]}},

    {"type":"Feature",
     "properties":{"uts:kind":"road","id":"AB","from":"A","to":"B",
                   "length":120,"speed_kmh":60,"capacity":18,"lanes":3,"oneway":false},
     "geometry":{"type":"LineString","coordinates":[[0,0],[120,0]]}},
    {"type":"Feature",
     "properties":{"uts:kind":"road","id":"BC","from":"B","to":"C",
                   "length":90,"speed_kmh":40,"capacity":9,"lanes":1,"oneway":true},
     "geometry":{"type":"LineString","coordinates":[[120,0],[120,90]]}},

    {"type":"Feature",
     "properties":{"uts:kind":"building","name":"City Hall","kind":"civic",
                   "levels":4,"node":"A"},
     "geometry":{"type":"Polygon",
                 "coordinates":[[[10,10],[30,10],[30,25],[10,25],[10,10]]]}},

    {"type":"Feature",
     "properties":{"uts:kind":"zone","name":"CITY CENTER","kind":"downtown"},
     "geometry":{"type":"Polygon",
                 "coordinates":[[[-20,-20],[160,-20],[160,130],[-20,130],[-20,-20]]]}}
  ]
})JSON";

// A scratch path that cleans itself up, so a failing assertion cannot leave a
// stale file behind for the next run.
struct TempFile {
    std::filesystem::path path;

    explicit TempFile(const std::string& stem)
        : path(std::filesystem::temp_directory_path() / ("uts_" + stem + ".geojson")) {
        remove();
    }
    ~TempFile() { remove(); }

    TempFile(const TempFile&) = delete;
    TempFile& operator=(const TempFile&) = delete;

    void remove() const {
        std::error_code ec;
        std::filesystem::remove(path, ec);
    }
    std::string str() const { return path.string(); }
};

// Load `text`, write the graph back out, and load the written file again.
// `out` ends up holding whatever survived the trip.
void round_trip(const std::string& text, TrafficGraph& out, const std::string& stem) {
    TrafficGraph source;
    MapLoader::load_from_geojson_string(source, text, "<test>");
    TempFile tmp(stem);
    MapLoader::write_geojson(source, tmp.str());
    MapLoader::load_from_geojson(out, tmp.str());
}

}  // namespace

TEST_CASE("write_geojson round-trips every feature kind") {
    TrafficGraph g;
    round_trip(kMap, g, "roundtrip");

    CHECK(g.intersections.size() == 3);
    CHECK(g.edges.size() == 3);  // AB, AB_r, BC
    CHECK(g.buildings.size() == 1);
    CHECK(g.zones.size() == 1);
}

TEST_CASE("nodes keep their label, zone and position") {
    TrafficGraph g;
    round_trip(kMap, g, "nodes");

    REQUIRE(g.intersections.count("A") == 1);
    const Intersection* a = g.intersections.at("A").get();
    CHECK(a->display_name == "Alpha & 1st");
    CHECK(a->zone == "downtown");
    CHECK(a->x == doctest::Approx(0.0));
    CHECK(a->y == doctest::Approx(0.0));

    REQUIRE(g.intersections.count("C") == 1);
    const Intersection* c = g.intersections.at("C").get();
    CHECK(c->x == doctest::Approx(120.0));
    CHECK(c->y == doctest::Approx(90.0));
    CHECK_FALSE(c->is_signalized());
}

TEST_CASE("a signalized node keeps its light timings") {
    TrafficGraph g;
    round_trip(kMap, g, "signal");

    REQUIRE(g.intersections.count("B") == 1);
    const Intersection* b = g.intersections.at("B").get();
    REQUIRE(b->is_signalized());
    REQUIRE(b->light_system != nullptr);
    CHECK(b->light_system->get_groups() == 3);
    CHECK(b->light_system->get_green_duration() == doctest::Approx(18.5));
    CHECK(b->light_system->get_yellow_duration() == doctest::Approx(4.0));
    CHECK(b->light_system->get_red_duration() == doctest::Approx(2.5));
}

TEST_CASE("a two-way street is written once and reloads as two roads") {
    TrafficGraph g;
    round_trip(kMap, g, "twoway");

    REQUIRE(g.edges.count("AB") == 1);
    REQUIRE(g.edges.count("AB_r") == 1);

    const Road* ab = g.edges.at("AB").get();
    CHECK(ab->source->name == "A");
    CHECK(ab->target->name == "B");
    CHECK(ab->get_lanes() == 3);
    CHECK(ab->capacity == 18);
    CHECK(ab->get_speed_limit() == doctest::Approx(60.0));
    CHECK(ab->length == doctest::Approx(120.0));
    CHECK_FALSE(ab->is_one_way);

    // The reverse twin is generated on load, not stored in the file.
    const Road* rev = g.edges.at("AB_r").get();
    CHECK(rev->source->name == "B");
    CHECK(rev->target->name == "A");
    CHECK(rev->get_lanes() == 3);
}

TEST_CASE("a one-way street stays one-way") {
    TrafficGraph g;
    round_trip(kMap, g, "oneway");

    REQUIRE(g.edges.count("BC") == 1);
    CHECK(g.edges.count("BC_r") == 0);

    const Road* bc = g.edges.at("BC").get();
    CHECK(bc->is_one_way);
    CHECK(bc->get_lanes() == 1);
    CHECK(bc->capacity == 9);
    CHECK(bc->get_speed_limit() == doctest::Approx(40.0));
    CHECK(bc->length == doctest::Approx(90.0));
}

TEST_CASE("a building keeps its footprint and its anchor node") {
    TrafficGraph g;
    round_trip(kMap, g, "building");

    REQUIRE(g.buildings.count("City Hall") == 1);
    const Building* b = g.buildings.at("City Hall").get();
    CHECK(b->get_type() == "civic");
    CHECK(b->get_levels() == 4);
    CHECK(b->get_node_id() == "A");

    // The writer closes the ring; the loader drops the repeated last point, so
    // the corner count has to come back unchanged.
    REQUIRE(b->get_footprint().size() == 4);
    CHECK(b->get_footprint()[0].first == doctest::Approx(10.0));
    CHECK(b->get_footprint()[0].second == doctest::Approx(10.0));
    CHECK(b->get_footprint()[2].first == doctest::Approx(30.0));
    CHECK(b->get_footprint()[2].second == doctest::Approx(25.0));
}

TEST_CASE("a zone keeps its bounding box") {
    TrafficGraph g;
    round_trip(kMap, g, "zone");

    REQUIRE(g.zones.size() == 1);
    const Zone& z = g.zones.front();
    CHECK(z.name == "CITY CENTER");
    CHECK(z.kind == "downtown");
    CHECK(z.x == doctest::Approx(-20.0));
    CHECK(z.y == doctest::Approx(-20.0));
    CHECK(z.w == doctest::Approx(180.0));
    CHECK(z.h == doctest::Approx(150.0));
}

TEST_CASE("a second round trip changes nothing") {
    TrafficGraph once;
    round_trip(kMap, once, "stable1");

    TempFile a("stable_a");
    TempFile b("stable_b");
    MapLoader::write_geojson(once, a.str());

    TrafficGraph twice;
    MapLoader::load_from_geojson(twice, a.str());
    MapLoader::write_geojson(twice, b.str());

    CHECK(twice.intersections.size() == once.intersections.size());
    CHECK(twice.edges.size() == once.edges.size());
    CHECK(twice.buildings.size() == once.buildings.size());
    CHECK(twice.zones.size() == once.zones.size());
}

TEST_CASE("an empty graph still writes a well-formed file") {
    TrafficGraph empty;
    TempFile tmp("empty");
    REQUIRE_NOTHROW(MapLoader::write_geojson(empty, tmp.str()));
    CHECK(std::filesystem::exists(tmp.path));

    // Reading it back is refused on purpose: load_from_geojson rejects a map
    // with no nodes, because there is nothing to drive on. What matters here is
    // that the "generator" stamp keeps the file on the native code path, so the
    // failure says what is actually wrong instead of blaming missing OSM tags.
    TrafficGraph reloaded;
    CHECK_THROWS_WITH_AS(MapLoader::load_from_geojson(reloaded, tmp.str()),
                         doctest::Contains("contains no nodes"), MapLoadError);
}

TEST_CASE("a path that cannot be opened raises MapLoadError") {
    TrafficGraph g;
    MapLoader::load_from_geojson_string(g, kMap, "<test>");

    const std::string bad =
        (std::filesystem::temp_directory_path() / "uts_no_such_dir" / "out.geojson").string();
    CHECK_THROWS_AS(MapLoader::write_geojson(g, bad), MapLoadError);
}
