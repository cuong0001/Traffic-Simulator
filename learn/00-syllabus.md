# Traffic Simulator — Course Syllabus

This is the written companion course to the codebase, generated per the rules in
[`MASTER_TUTOR_PROMPT.md`](../MASTER_TUTOR_PROMPT.md): every module below explains
its files **line by line**, covering **Meaning → Why → Usage** for each chunk of
code, grounded in the actual source (not a paraphrase of it).

Read the modules in order — later modules assume the vocabulary and classes
introduced earlier.

| # | Module | Files covered |
|---|--------|----------------|
| 01 | [Orientation](01-orientation.md) | `CMakeLists.txt` |
| 02 | [Core graph model](02-core-graph-model.md) | `core/Graph`, `core/Intersection`, `core/Road`, `core/Building`, `core/Zone` |
| 03 | [Traffic control primitives](03-traffic-lights.md) | `core/TrafficLight` |
| 04 | [Routing / Strategy Pattern](04-routing.md) | `routing/PathFinding`, `routing/BFS`, `routing/Dijkstra`, `routing/AStar` |
| 05 | [Vehicles](05-vehicles.md) | `vehicles/Vehicle`, `vehicles/VehicleTypes` (Car, Bus, EmergencyVehicle, Motorbike) |
| 06 | [Events and Observer wiring](06-events-observer.md) | `utils/Observer`, `events/TrafficEvent`, `simulation/TrafficController` |
| 07 | [The simulation core](07-simulation-core.md) | `simulation/TrafficSimulator` (Singleton tick loop), `simulation/JunctionArbiter` |
| 08 | [Supporting systems](08-supporting-systems.md) | `utils/MapLoader`, `utils/Json`, `simulation/MapEditor`, `simulation/HistoryStore`, `utils/StatisticsManager` |
| 09 | [Server and entrypoint](09-server-and-entrypoint.md) | `server/WebSocketServer`, `cpp_backend/src/main.cpp` |
| 10 | [Synthesis](10-synthesis.md) | End-to-end trace through every module above, revisited against the known limitations |

## Known limitations kept in view throughout

- Vehicles are locked to discrete lanes (no continuous lateral position).
- Junction overlapping can happen.
- Gridlocks are handled reactively (detected and broken up after the fact), not prevented proactively.
- The tick loop is single-threaded.

These are called out explicitly wherever the responsible code is discussed, rather
than being presented as flawless design.
