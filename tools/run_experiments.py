#!/usr/bin/env python3
"""
Compare BFS, Dijkstra and A* inside a running traffic simulation via its REST
API. Records per-run stat deltas (trips, travel time, congestion, recomputes)
across four traffic scenarios and writes a CSV, three charts, and a markdown
report.

The simulator must already be running on --host/--port (default localhost:8080).
Charts need matplotlib (pip install matplotlib); the CSV is written regardless.

  python tools/run_experiments.py            # full 4 x 3 = 12 runs
  python tools/run_experiments.py --smoke     # 1 run, 60s sim, quick check
"""

import argparse
import csv
import json
import os
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from datetime import datetime

HOST = "localhost"
PORT = 8080
SIM_SECONDS = 300        # simulated seconds per run
SPEED = 10               # sim-time / real-time via /api/speed
POLL_INTERVAL = 2.0      # real seconds between /api/state polls
RESULTS_CSV_STEM = "tools/results"
REPORT_MD = "docs/experiments.md"
CHARTS = [
    ("completed_trips",   "Completed trips",     "trips",            "trips"),
    ("avg_travel_time_s", "Average travel time", "seconds",          "traveltime"),
    ("avg_congestion",    "Average congestion",  "congestion (0-1)", "congestion"),
]

ALGORITHMS = ["bfs", "dijkstra", "astar"]
SCENARIOS = [
    {"key": "low-traffic", "label": "Low traffic (target=40)",  "target": 40,  "closure": False},
    {"key": "rush-hour",    "label": "Rush hour (target=200)",  "target": 200, "closure": False},
    {"key": "closure",      "label": "Closed road (random)",    "target": 120, "closure": True},
    {"key": "sustained",    "label": "Sustained auto-spawn",    "target": 120, "closure": False},
]


def api_get(base, path, params=None, timeout=30):
    qs = "?" + urllib.parse.urlencode(params) if params else ""
    url = base + path + qs
    try:
        with urllib.request.urlopen(url, timeout=timeout) as resp:
            body = resp.read().decode("utf-8", "replace")
    except urllib.error.URLError as e:
        raise RuntimeError(
            f"cannot reach {url}: {e.reason}\n"
            "Is the simulator running? Start it with e.g.\n"
            "  build_release\\traffic_simulator.exe data\\maps\\small_map.geojson 8080"
        ) from e
    try:
        return json.loads(body)
    except json.JSONDecodeError:
        raise RuntimeError(f"non-JSON from {url}: {body[:200]}")


def snapshot(state):
    s = state.get("stats", {}) or {}
    history = s.get("history") or []
    return {
        "time": state.get("time", 0.0),
        "trips": s.get("completedTrips", 0),
        "computations": s.get("routeComputations", 0),
        "avg_travel": s.get("avgTravelTime", 0.0),
        "active": state.get("active", 0),
        "congestion": history[-1].get("congestion", 0.0) if history else 0.0,
    }


def run_one(base, sc, algo, sim_seconds, poll):
    api_get(base, "/api/clearVehicles")
    api_get(base, "/api/speed", {"x": str(SPEED)})
    api_get(base, "/api/auto", {"on": "1"})
    api_get(base, "/api/target", {"value": str(sc["target"])})
    if sc["closure"]:
        api_get(base, "/api/event", {"type": "closure"})  # random road
    api_get(base, "/api/algorithm", {"name": algo})

    start = snapshot(api_get(base, "/api/state"))
    t0 = start["time"]
    samples = []
    deadline = time.monotonic() + (sim_seconds / SPEED) * 3 + 30

    while True:
        state = api_get(base, "/api/state")
        snap = snapshot(state)
        samples.append(snap["congestion"])
        if snap["time"] - t0 >= sim_seconds:
            break
        if time.monotonic() > deadline:
            print(f"  [warn] sim stalled at {snap['time'] - t0:.0f}/{sim_seconds}s",
                  file=sys.stderr)
            break
        time.sleep(poll)
    end = snapshot(state)

    trips = end["trips"] - start["trips"]
    # avgTravelTime is cumulative -> reconstruct per-run average.
    total0 = start["avg_travel"] * start["trips"]
    total1 = end["avg_travel"] * end["trips"]
    avg_travel = (total1 - total0) / trips if trips else 0.0

    return {
        "scenario": sc["key"],
        "algorithm": algo,
        "sim_seconds": round(end["time"] - t0, 1),
        "completed_trips": trips,
        "avg_travel_time_s": round(avg_travel, 2),
        "avg_congestion": round(sum(samples) / len(samples), 3) if samples else 0.0,
        "route_recomputations": end["computations"] - start["computations"],
        "active_vehicles_end": end["active"],
    }


def write_csv(rows, run_id, ts_header):
    path = f"tools/results_{run_id}.csv"
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["timestamp", "scenario", "algorithm", "sim_seconds",
                    "completed_trips", "avg_travel_time_s", "avg_congestion",
                    "route_recomputations", "active_vehicles_end"])
        for r in rows:
            w.writerow([ts_header, r["scenario"], r["algorithm"], r["sim_seconds"],
                        r["completed_trips"], r["avg_travel_time_s"],
                        r["avg_congestion"], r["route_recomputations"],
                        r["active_vehicles_end"]])
    return path


def write_charts(rows, run_id):
    """Write 3 bar charts with timestamped filenames. False if matplotlib missing."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed -- skipping charts", file=sys.stderr)
        return False

    labels = [sc["label"] for sc in SCENARIOS]
    by = {sc["key"]: {} for sc in SCENARIOS}
    for r in rows:
        by[r["scenario"]][r["algorithm"]] = r

    for metric, title, ylabel, stem in CHARTS:
        fig, ax = plt.subplots(figsize=(9, 5))
        for i, algo in enumerate(ALGORITHMS):
            vals = [by[sc["key"]].get(algo, {}).get(metric, 0) for sc in SCENARIOS]
            ax.bar([x + i * 0.25 for x in range(len(SCENARIOS))],
                   vals, 0.25, label=algo)
        ax.set_xticks([x + 0.25 for x in range(len(SCENARIOS))])
        ax.set_xticklabels(labels, rotation=15, ha="right")
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.legend(title="Algorithm")
        fig.tight_layout()
        os.makedirs(f"docs/images/{run_id}", exist_ok=True)
        fig.savefig(f"docs/images/{run_id}/experiments_{stem}.png", dpi=120)
        plt.close(fig)
    return True


def write_report(rows, charts_ok, run_id, ts_header):
    """Append a new dated section with the results table to the markdown report."""
    table_lines = [
        "| Scenario | Algorithm | Trips | Avg travel (s) | Avg congestion | Recomputes | Active end |",
        "|---|---|---|---|---|---|---|",
    ]
    for sc in SCENARIOS:
        for r in sorted([x for x in rows if x["scenario"] == sc["key"]],
                        key=lambda x: ALGORITHMS.index(x["algorithm"])):
            table_lines.append(
                f"| {sc['label']} | {r['algorithm']} | {r['completed_trips']} | "
                f"{r['avg_travel_time_s']} | {r['avg_congestion']} | "
                f"{r['route_recomputations']} | {r['active_vehicles_end']} |")
    table = "\n".join(table_lines)

    charts_section = ""
    if charts_ok:
        chart_block = "".join(
            f"![{title}](images/{run_id}/experiments_{stem}.png)\n\n"
            for _, title, _, stem in CHARTS)
        charts_section = f"### Charts\n\n{chart_block}"

    section = f"""## {ts_header}

Four scenarios x three algorithms; each run advanced {SIM_SECONDS} s of simulated time (x{SPEED} speed). Closure picks a random road each run.

### Results

{table}

{charts_section}"""

    try:
        with open(REPORT_MD, "r", encoding="utf-8") as f:
            existing = f.read()
    except FileNotFoundError:
        existing = ""

    if existing:
        sep = "\n" if existing.endswith("\n") else "\n\n"
        content = existing + sep + section + "\n"
    else:
        content = section + "\n"
    with open(REPORT_MD, "w", encoding="utf-8") as f:
        f.write(content)


def main():
    p = argparse.ArgumentParser(
        description="Compare routing algorithms in a running simulation.")
    p.add_argument("--host", default=HOST)
    p.add_argument("--port", type=int, default=PORT)
    p.add_argument("--sim-seconds", type=int, default=SIM_SECONDS)
    p.add_argument("--poll-interval", type=float, default=POLL_INTERVAL)
    p.add_argument("--smoke", action="store_true", help="quick 1-run check")
    args = p.parse_args()

    base = f"http://{args.host}:{args.port}"

    if args.smoke:
        runs = [(SCENARIOS[0], "dijkstra", 60)]
        print("[smoke] 1 run x 60s sim")
    else:
        runs = [(sc, a, args.sim_seconds) for sc in SCENARIOS for a in ALGORITHMS]
        print(f"{len(runs)} runs, ~{len(runs) * args.sim_seconds / SPEED:.0f}s real at x{SPEED}.")

    try:
        api_get(base, "/api/state", timeout=5)
    except RuntimeError as e:
        print(str(e), file=sys.stderr)
        return 2

    rows = []
    for i, (sc, algo, secs) in enumerate(runs, 1):
        print(f"[{i}/{len(runs)}] {sc['key']} / {algo} ...", end=" ", flush=True)
        row = run_one(base, sc, algo, secs, args.poll_interval)
        rows.append(row)
        print(f"trips={row['completed_trips']}  recalc={row['route_recomputations']}  "
              f"avgT={row['avg_travel_time_s']}s  jam={row['avg_congestion']}  "
              f"active={row['active_vehicles_end']}")

    now = datetime.now()
    run_id = now.strftime("%Y-%m-%d_%H-%M")
    ts_header = now.strftime("%Y-%m-%d %H:%M")

    csv_path = write_csv(rows, run_id, ts_header)
    print(f"wrote {csv_path}")

    charts_ok = write_charts(rows, run_id)
    if charts_ok:
        print("wrote 3 charts in docs/")

    write_report(rows, charts_ok, run_id, ts_header)
    print(f"appended to {REPORT_MD}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
