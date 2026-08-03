# Traffic Simulator

This project is a part of the CSC10003 - Object-Oriented Programming course at
VNUHCM - University of Science. It simulates urban traffic in real life: a
C++ backend runs a graph-based road network with routing, moving vehicles,
traffic lights and dynamic events; a browser frontend polls it and draws the
city live.

## What's here

```
cpp_backend/
  include/, src/
    core/        Graph, Intersection, Road, Building, TrafficLight, Zone --
                 the road network itself.
    vehicles/    Vehicle (abstract) and the four concrete types
                 (Car/Bus/EmergencyVehicle/Motorbike) + VehicleFactory.
    routing/     PathFinding (Strategy interface) and its three
                 implementations: BFS, Dijkstra, AStar.
    events/      TrafficEvent (abstract) and its four concrete kinds
                 (congestion, accident, closure, broken light).
    simulation/  TrafficSimulator (the tick loop + live map editing +
                 state serialization; a Singleton) and TrafficController
                 (owns events, the Observer Subject vehicles attach to).
    utils/       MapLoader (GeoJSON <-> graph), StatisticsManager, a small
                 hand-rolled JSON writer.
    server/      A small dependency-free HTTP server exposing the REST API
                 and serving the static frontend.
  src/main.cpp   Entry point: loads a map, starts the tick thread and the
                 HTTP server.

web_frontend/    Static HTML/CSS/JS viewer + control panel. Polls the
                 backend's REST API and renders the city on a canvas.

data/maps/       GeoJSON road networks (small_map.geojson is the default).

tests/           unit_tests/, edge_cases/, stress_tests/ -- see "Testing"
                 below.

docs/            Working notes and plan documents (gitignored; not part of
                 the graded deliverable, kept locally for the team).
```

See [docs/architecture.md](docs/architecture.md) for the class diagram and a
longer description of how the pieces fit together, and which design pattern
each one demonstrates.

## Building and running

Needs CMake (>= 3.16) and a C++20 compiler. On Windows, Ninja + MinGW-w64;
on Linux/macOS, any recent GCC/Clang works the same way.

```
cmake -S . -B build -G Ninja
cmake --build build
build/traffic_simulator data/maps/small_map.geojson 8080
```

(On Windows that last line is `build\traffic_simulator.exe ...`; `run.bat`
in the repo root does all three steps in one go and opens a browser.)

Then open **http://localhost:8080** — vehicles should be moving within a
second or two, signals cycling, and the menu bar in the top-left opens the
control sidebar (spawn vehicles, trigger events, edit the map live, save/load
a city).

`traffic_simulator [map.geojson] [port]` — both arguments are optional and
default to `data/maps/small_map.geojson` and `8080`.

## The REST API

Everything the frontend does goes through `GET`/`POST` endpoints under
`/api/...` — map data, live simulation state (JSON and a compact binary
form), playback control (pause/resume/speed), spawning and removing
vehicles, triggering traffic events, and live map editing (add/remove a
road, junction or building; save/load the whole map). `main.cpp` is the
single place that lists them; `web_frontend/js/api.js` is the client-side
wrapper around the same list.

## Testing

```
cmake --build build --target test_simulation_unit   # or any other test_* target
build/test_simulation_unit
```

(Run from the repo root — several tests resolve `data/maps/...` as a
relative path.) Three tiers, under `tests/`:

- **`unit_tests/`** — one executable per subsystem (`test_pathfinder`,
  `test_vehicle`, `test_maploader`, `test_simulation`, `test_signals`,
  `test_trafficevent`, `test_statistics`, plus a dedicated
  `test_gridlock_stress` for four-way unsignalized junctions).
- **`edge_cases/`** — invalid map files, disconnected graphs, unreachable
  destinations, empty graphs.
- **`stress_tests/`** — larger networks, many vehicles.

`CMakeLists.txt` registers one executable per test file; `cmake --build
build` builds all of them alongside the simulator.
