# Output from tools/route_bench.cpp:
```
[map] pruned 202 node(s) outside the largest connected road network (kept 2617)
[map] signalized 3 junction(s) from OSM traffic_signals (9 skipped: nothing to arbitrate)
map           : data\maps\small_map.geojson
graph         : 2617 intersections, 6097 roads
pairs         : 100 attempted, 100 solved by all three

Algorithm    Solved  Nodes/route    Time (us)     Route cost      Roads
----------   ------  -----------    ---------     ----------      -----
BFS             100       1406.5        450.3          157.8       36.0
Dijkstra        100       1363.8        672.2          116.8       42.0
A*              100        518.7        405.5          116.8       42.0

--- What the table shows ---
A* == Dijkstra cost : 100 / 100 pairs   (admissible heuristic => optimal)
A* search effort    : 38.0% of Dijkstra's nodes
BFS route cost      : 135.1% of Dijkstra's   (more expensive: BFS counts roads, not weight)
```
- Note that us here means microseconds.
- Explaination : #TODO



