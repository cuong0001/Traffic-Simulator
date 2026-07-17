#!/usr/bin/env python3
"""Convert the three CSV files (nodes / roads / buildings) into a SINGLE
GeoJSON file in the "uts:kind" format that MapLoader reads.

Usage:
    python tools/csv_to_geojson.py data/maps data/maps/downtown.geojson

- Opposite-direction road pairs (a_b and b_a) collapse into one oneway=false
  feature.
- Each building gets a square footprint around the node it is anchored to.
"""
import csv
import json
import os
import sys

BUILDING_HALF = 180.0  # half-side of the square building footprint (meters)


def load_nodes(path):
    nodes = {}
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            nodes[row["id"]] = (float(row["x"]), float(row["y"]))
    return nodes


def load_roads(path):
    """Return a list of (id, from, to, lanes, speed, oneway) with both
    directions merged."""
    rows = []
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            rows.append(row)

    seen_pairs = set()  # frozenset({from, to}) already emitted
    roads = []
    by_pair = {}
    for r in rows:
        by_pair.setdefault(frozenset((r["source"], r["target"])), []).append(r)

    for r in rows:
        pair = frozenset((r["source"], r["target"]))
        if pair in seen_pairs:
            continue
        seen_pairs.add(pair)
        # Two-way exists iff the reverse edge is present.
        twoway = any(o["source"] == r["target"] and o["target"] == r["source"]
                     for o in by_pair[pair])
        roads.append({
            "id": f"{r['source']}_{r['target']}",
            "from": r["source"],
            "to": r["target"],
            "lanes": int(r["lanes"]),
            "speed_kmh": float(r["speed_limit"]),
            "oneway": (r["is_one_way"] == "1") or not twoway,
        })
    return roads


def load_buildings(path, nodes):
    buildings = []
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            nid = row["node_id"]
            if nid not in nodes:
                continue
            buildings.append({
                "name": row["name"],
                "kind": row["type"],
                "node": nid,
                "center": nodes[nid],
            })
    return buildings


def square(cx, cy, half):
    return [[cx - half, cy - half], [cx + half, cy - half],
            [cx + half, cy + half], [cx - half, cy + half], [cx - half, cy - half]]


def build(maps_dir):
    nodes = load_nodes(os.path.join(maps_dir, "nodes.csv"))
    roads = load_roads(os.path.join(maps_dir, "roads.csv"))
    buildings = load_buildings(os.path.join(maps_dir, "buildings.csv"), nodes)

    features = []
    for nid, (x, y) in nodes.items():
        features.append({
            "type": "Feature",
            "properties": {"uts:kind": "node", "id": nid, "name": nid, "zone": ""},
            "geometry": {"type": "Point", "coordinates": [x, y]},
        })
    for r in roads:
        a, b = nodes[r["from"]], nodes[r["to"]]
        features.append({
            "type": "Feature",
            "properties": {"uts:kind": "road", "id": r["id"], "from": r["from"],
                           "to": r["to"], "speed_kmh": r["speed_kmh"],
                           "lanes": r["lanes"], "oneway": r["oneway"]},
            "geometry": {"type": "LineString", "coordinates": [list(a), list(b)]},
        })
    for bldg in buildings:
        cx, cy = bldg["center"]
        features.append({
            "type": "Feature",
            "properties": {"uts:kind": "building", "name": bldg["name"],
                           "kind": bldg["kind"], "levels": 1, "node": bldg["node"]},
            "geometry": {"type": "Polygon", "coordinates": [square(cx, cy, BUILDING_HALF)]},
        })

    return {"type": "FeatureCollection", "generator": "csv_to_geojson.py",
            "features": features}


def main():
    maps_dir = sys.argv[1] if len(sys.argv) > 1 else "data/maps"
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(maps_dir, "downtown.geojson")
    doc = build(maps_dir)
    with open(out, "w", encoding="utf-8") as f:
        json.dump(doc, f, indent=1, ensure_ascii=False)
    counts = {}
    for ft in doc["features"]:
        k = ft["properties"]["uts:kind"]
        counts[k] = counts.get(k, 0) + 1
    print(f"Wrote {out}: {counts}")


if __name__ == "__main__":
    main()
