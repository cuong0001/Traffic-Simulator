# Benchmark experiments

This report is about the two separate benchmarks that measure the routing work. `route_bench.cpp` that runs BFS, Dijkstra and A* over the same fixed origin and destination pairs on a static map with no traffic, and `run_experiments.py` that runs the same three algorithms inside a live simulation through the REST API, under four traffic scenarios, and records per-run deltas of the simulator's statistics. The offline benchmark establishes what each algorithm is on a clean graph, while the simulation benchmark shows what each algorithm does when the network is actually moving. 

## The offline benchmark

`route_bench.cpp` runs 100 fixed pairs and reports, per algorithm, how many pairs it solved, how many nodes it settled, the route cost, and how many roads the route used. Routes are priced by time, so the cost column is travel time in seconds.

| Algorithm | Solved | Nodes/route | Route cost (s) | Roads |
|---|---|---|---|---|
| BFS | 100 | 1406.5 | 157.8 | 36.0 |
| Dijkstra | 100 | 1363.8 | 116.8 | 42.0 |
| A* | 100 | 518.7 | 116.8 | 42.0 |

The table carries two facts. BFS uses the fewest roads, 36.0 against 42.0 for the other two, yet its routes cost the most, 157.8 seconds against 116.8. The reason is that BFS has no notion of edge weight. It expands the graph by layer and takes the first route that reaches the goal, so it minimises the number of road segments rather than the travel time. The fewest-hop path tends to leave the fast main roads for slow local streets, and the extra seconds are the cost of ignoring speed limits.

A* and Dijkstra return the same cost, because both minimise the same cost function and both are guaranteed to find the optimal route. Dijkstra expands outward in every direction from the start and A* does the same but uses its heuristic to aim at the goal, so it skips the parts of the graph Dijkstra expands without need. Both converge on the same optimal route, so the cost is identical and only the amount of work differs. A* settles 518.7 nodes per route against Dijkstra's 1363.8, about 38 percent of the work. The heuristic hands A* a lower bound on the remaining cost, so it expands the node that looks closest to the goal first and avoids expanding the rest.

The timing column is not shown in this report because timing is noisy and machine dependent. It shifts with the CPU, the background load and the optimisation level, while node count is a property of the algorithm and does not. The offline experiment therefore reports node counts as the stable number and treats timing as a rough check.

## The simulation benchmark

`run_experiments.py` drives a running simulator over the REST API. It sets the scenario dials, picks one algorithm, warms the network up to its target, lets the simulation run for 300 simulated seconds, and records the delta in the simulator's statistics. Every scenario and algorithm combination runs three times, and the result is reported as a mean plus a range. 

The four scenarios are low traffic with a target of 40 vehicles, rush hour with a target of 200, a closed road with a random road picked each run, and sustained auto-spawn with a target of 120. Random events are turned off for the runs that are not about closures, so a stray accident does not move the numbers. Per run it records trips per sim minute, average travel time, congestion, recomputes per sim minute, nodes and microseconds per recompute, and fleet velocity. The nodes and microseconds come from the routing counter, which the simulator now feeds into its statistics at both spawn and replan.

### Results (mean of three repeats as of 2026-08-03)

| Scenario | Algorithm | Trips/min | Travel (s) | Nodes/recompute | Velocity (m/s) |
|---|---|---|---|---|---|
| Low traffic | BFS | 11.4 | 216 | 1398 | 7.01 |
| Low traffic | Dijkstra | 13.7 | 175 | 1410 | 8.71 |
| Low traffic | A* | 14.3 | 163 | 589 | 8.86 |
| Rush hour | BFS | 53.6 | 220 | 1319 | 7.03 |
| Rush hour | Dijkstra | 67.6 | 182 | 1337 | 8.16 |
| Rush hour | A* | 71.1 | 172 | 574 | 8.29 |
| Closed road | BFS | 32.2 | 214 | 1312 | 6.59 |
| Closed road | Dijkstra | 43.3 | 172 | 1338 | 8.06 |
| Closed road | A* | 42.1 | 169 | 578 | 8.57 |
| Sustained | BFS | 33.0 | 214 | 1361 | 7.17 |
| Sustained | Dijkstra | 42.9 | 178 | 1326 | 8.30 |
| Sustained | A* | 45.0 | 161 | 547 | 8.61 |

The pattern is consistent across all four scenarios, and the reasons match the offline benchmark results. BFS moves the fewest trips per minute and its trips take the longest, because it routes vehicles onto the fewest-hop paths, which ignore speed limits and congestion. Under time routing that means slower roads and longer trips, so fewer trips complete per minute and the fleet's mean speed is lower. Dijkstra and A* are close on trips and travel time, because they issue the same routes, and the traffic outcome is a property of the routes rather than of the algorithm that found them as the small differences between the two are repeat noise. The algorithms separate on search cost, where A* explores roughly 550 to 590 nodes per recompute against 1310 to 1410 for Dijkstra and BFS, about two fifths of Dijkstra's work. That ratio is stable across empty and jammed networks, because the heuristic prunes the same way regardless of load.

### The search-cost chart

![Nodes explored per recompute](../docs/images/2026-08-03_16-06/experiments_nodes.png)

The nodes-per-recompute chart shows why search cost is the metric that separates the algorithms. Dijkstra and BFS both expand the graph broadly and end up above 1300 nodes in every scenario, while A*'s heuristic concentrates the search near the route and keeps it near 550 to 590. The gap does not change much between scenarios because the traffic outcome cannot separate the algorithms, since they drive the same roads, but the work each one does to find those routes is a direct property of the algorithm and independent of the traffic model.

### The throughput chart

![Trips per sim-minute](../docs/images/2026-08-03_16-06/experiments_trips.png)

The trips-per-minute chart shows the traffic outcome. BFS is the lowest bar in every scenario, for the same reason as the offline benchmark: its routes take longer, so fewer trips finish per minute. Dijkstra and A* sit close together and above BFS, and their difference is small and not consistent in direction, which follows from them issuing the same routes. The chart also shows that throughput tracks demand, since rush hour moves far more trips than low traffic because the target is much higher while the height of the bars is mostly set by the scenario rather than by the algorithm.

### The velocity chart

![Network velocity over time, rush hour](../docs/images/2026-08-03_16-06/experiments_velocity_rush-hour.png)

BFS keeps the fleet slower than the other two for most of the rush hour run, around 6.9 m/s against 8.2 m/s. The reason is the same one that shows up in the trips and travel time numbers. BFS chooses the fewest-hop path, which ignores speed limits, so its vehicles are spread across the slower local roads and the fleet-wide mean speed is pulled down. Dijkstra and A*, which route by travel time, keep their vehicles on the faster roads, so the fleet moves faster overall. The two lines sit together because both algorithms pick the same fastest routes.

The speed dips and rises over the run because traffic is not steady. Congestion forms when too many vehicles reach a junction at once, they slow or stop, the queue pushes back, and the speed recovers once the jam clears. The dips are bigger in rush hour than in low traffic, because the roads are near capacity with a target of 200 vehicles, so a small disruption turns into a larger slowdown with 40 vehicles there is room for the traffic to spread out and absorb it. The gap between BFS and the others also widens at the worst moments, because BFS's slower routes mean its vehicles are the ones caught in the congestion and they take longer to clear it.

## Conclusion
The offline benchmark is the reproducible baseline and the regression safety net, while the simulation benchmark shows the same behaviour against real traffic, jams and closures.

The two benchmarks agree on the ranking, and for the same reasons. BFS returns a different route than Dijkstra and A*, because it minimises the number of road segments where the other two minimise cost, and fewer segments is not shorter: the fewest-hop path ignores speed limits and congestion. In the offline table BFS uses the fewest roads yet the most expensive route, and in the simulation it is the slowest on trips, travel time and velocity. A* settles far fewer nodes than Dijkstra while returning the same route, because the heuristic concentrates the search toward the goal, it does about two fifths of Dijkstra's work, and the ratio holds under every scenario. Node count is the trustworthy measurment here, as because the timing shifts with the machine and the optimisation level while node count is a property of the algorith, plus the simulation's trip counts are noisy, and the search cost is the number that separates the algorithms. 