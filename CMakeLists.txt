cmake_minimum_required(VERSION 3.16)
project(TrafficSimulator CXX)

# Default to Release when the caller did not ask for a build type.
#
# Without this, the command everyone actually types --
#
#     cmake -S . -B build -G Ninja
#
# leaves CMAKE_BUILD_TYPE empty, which means NO optimisation flags at all: not
# -O2, not even -O0 explicitly. The binary still runs and every test still
# passes, so nothing looks wrong; it is only several times slower. That makes
# every timing anyone measures wrong, and wrong in a direction that flatters
# whatever they just changed. One benchmark in this project was reported as
# "13x faster" on exactly that mistake -- the honest figure was 3.7x.
#
# Multi-config generators (Visual Studio, Xcode) pick the configuration at
# build time instead, so leave them alone.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING
        "Build type: Debug, Release, RelWithDebInfo or MinSizeRel" FORCE)
    message(STATUS "No build type given, defaulting to Release. "
                   "Use -DCMAKE_BUILD_TYPE=Debug for an unoptimised build.")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
    add_compile_options(/W3)
else()
    add_compile_options(-Wall -Wextra)
endif()

# Implemented source files (empty stubs are omitted so the build does not fail
# at link time).
set(BACKEND_SOURCES
    cpp_backend/src/core/Graph.cpp
    cpp_backend/src/core/Intersection.cpp
    cpp_backend/src/core/Road.cpp
    cpp_backend/src/core/Building.cpp
    cpp_backend/src/core/TrafficLight.cpp
    cpp_backend/src/routing/Dijkstra.cpp
    cpp_backend/src/routing/BFS.cpp
    cpp_backend/src/routing/AStar.cpp
    cpp_backend/src/utils/MapLoader.cpp
    cpp_backend/src/server/WebSocketServer.cpp
    cpp_backend/src/utils/StatisticsManager.cpp
    cpp_backend/src/utils/Json.cpp
    cpp_backend/src/utils/Observer.cpp
    cpp_backend/src/events/TrafficEvent.cpp
    cpp_backend/src/vehicles/Vehicle.cpp
    cpp_backend/src/vehicles/VehicleTypes.cpp
    cpp_backend/src/simulation/TrafficController.cpp
    cpp_backend/src/simulation/TrafficSimulator.cpp
)

find_package(Threads REQUIRED)

add_library(traffic_core STATIC ${BACKEND_SOURCES})
target_include_directories(traffic_core PUBLIC cpp_backend/include)
target_link_libraries(traffic_core PUBLIC Threads::Threads)
if(WIN32)
    target_link_libraries(traffic_core PUBLIC ws2_32)  # Winsock for the HTTP server
endif()

add_executable(traffic_simulator cpp_backend/src/main.cpp)
target_link_libraries(traffic_simulator PRIVATE traffic_core)

# Tests
add_executable(test_pathfinder_unit  tests/unit_tests/test_pathfinder.cpp)
target_link_libraries(test_pathfinder_unit PRIVATE traffic_core)

add_executable(test_pathfinder_edge  tests/edge_cases/test_pathfinder.cpp)
target_link_libraries(test_pathfinder_edge PRIVATE traffic_core)

#StatisticManager test==============================================================
#() Unit test
add_executable(test_statistics_unit tests/unit_tests/test_statistics.cpp)
target_link_libraries(test_statistics_unit PRIVATE traffic_core)

#() Edge case test
add_executable(test_statistics_edge tests/edge_cases/test_statistics.cpp)
target_link_libraries(test_statistics_edge PRIVATE traffic_core)

#() Stress test
add_executable(test_statistics_stress tests/stress_tests/test_statistics.cpp)
target_link_libraries(test_statistics_stress PRIVATE traffic_core)

#Traffic Event test===================================================================
add_executable(test_trafficevent_unit tests/unit_tests/test_trafficevent.cpp)
target_link_libraries(test_trafficevent_unit PRIVATE traffic_core)

#Vehicle test=========================================================================
add_executable(test_vehicle_unit tests/unit_tests/test_vehicle.cpp)
target_link_libraries(test_vehicle_unit PRIVATE traffic_core)

#Map loader test======================================================================
add_executable(test_maploader_unit tests/unit_tests/test_maploader.cpp)
target_link_libraries(test_maploader_unit PRIVATE traffic_core)

#Simulator test=======================================================================
add_executable(test_simulation_unit tests/unit_tests/test_simulation.cpp)
target_link_libraries(test_simulation_unit PRIVATE traffic_core)
#Traffic signal test==================================================================
add_executable(test_signals_unit tests/unit_tests/test_signals.cpp)
target_link_libraries(test_signals_unit PRIVATE traffic_core)
