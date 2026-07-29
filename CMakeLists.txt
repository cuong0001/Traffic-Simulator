cmake_minimum_required(VERSION 3.16)
project(TrafficSimulator CXX)

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
    cpp_backend/src/vehicles/Vehicle.cpp
    cpp_backend/src/vehicles/VehicleTypes.cpp
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

#StatisticManager test
#() Khai báo test unit
add_executable(test_statistics_unit tests/unit_tests/test_statistics.cpp)
target_link_libraries(test_statistics_unit PRIVATE traffic_core)

#() Khai báo test edge cases
add_executable(test_statistics_edge tests/edge_cases/test_statistics.cpp)
target_link_libraries(test_statistics_edge PRIVATE traffic_core)

#() Khai báo test stress
add_executable(test_statistics_stress tests/stress_tests/test_statistics.cpp)
target_link_libraries(test_statistics_stress PRIVATE traffic_core)
