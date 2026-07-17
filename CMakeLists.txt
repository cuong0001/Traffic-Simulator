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
    cpp_backend/src/graph/Graph.cpp
    cpp_backend/src/graph/Node.cpp
    cpp_backend/src/graph/Road.cpp
    cpp_backend/src/graph/Building.cpp
    cpp_backend/src/graph/TrafficLight.cpp
    cpp_backend/src/navigation/Dijkstra.cpp
    cpp_backend/src/utility/MapLoader.cpp
)

add_library(traffic_core STATIC ${BACKEND_SOURCES})
target_include_directories(traffic_core PUBLIC cpp_backend/include)

add_executable(traffic_simulator cpp_backend/main.cpp)
target_link_libraries(traffic_simulator PRIVATE traffic_core)
