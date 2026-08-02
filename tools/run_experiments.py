#!/usr/bin/env python3
"""
Compare BFS, Dijkstra and A* inside a running traffic simulation via its REST
API. Records per-run stat deltas (trips, travel time, congestion, recomputes,
search effort) across four traffic scenarios and writes a CSV, charts, and a
markdown report.

The simulator must already be running on --host/--port (default localhost:8080).
Charts need matplotlib (pip install matplotlib); the CSV is written regardless.

  python tools/run_experiments.py            # full 4 x 3 x 3 = 36 runs
  python tools/run_experiments.py --smoke     # 1 run, quick check
  python tools/run_experiments.py --repeats 5  # 5 repeats per combo
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
REPORT_MD = "docs/experiments.md"
CHARTS = [
    ("trips_per_sim_min",      "Trips per sim-minute",   "trips/min",        "trips"),
    ("avg_travel_time_s",      "Average travel time",    "seconds",          "traveltime"),
    ("avg_congestion",         "Average congestion",     "congestion (0-1)", "congestion"),
    ("avg_nodes_per_recompute","Avg nodes explored",     "nodes",            "nodes"),
    ("avg_micros_per_recompute","Avg compute time",      "microseconds",     "micros"),
]

ALGORITHMS = ["bfs", "dijkstra", "astar"]
SCENARIOS = [
    {"key": "low-traffic", "label": "Low traffic (target=40)",  "target": 40,  "closure": False},
    {"key": "rush-hour",    "label": "Rush hour (target=200)",  "target": 200, "closure": False},
    {"key": "closure",      "label": "Closed road (random)",    "target": 120, "closure": True},
    {"key": "sustained",    "label": "Sustained auto-spawn",    "target": 120, "closure": False},
]

METRICS = ["sim_seconds", "completed_trips", "trips_per_sim_min",
           "avg_travel_time_s", "avg_congestion", "route_recomputations",
           "recomputes_per_sim_min", "active_vehicles_end",
           "avg_nodes_per_recompute", "avg_micros_per_recompute"]


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


def snapshot(state, algo=None):
    s = state.get("stats", {}) or {}
    history = s.get("history") or []
    snap = {
        "time": state.get("time", 0.0),
        "trips": s.get("completedTrips", 0),
        "computations": s.get("routeComputations", 0),
        "avg_travel": s.get("avgTravelTime", 0.0),
        "active": state.get("active", 0),
        "congestion": history[-1].get("congestion", 0.0) if history else 0.0,
    }
    if algo:
        alg = (s.get("algorithms") or {}).get(algo) or {}
        snap["algo_computations"] = alg.get("computations", 0)
        snap["algo_avg_micros"] = alg.get("avgMicros", 0.0)
        snap["algo_avg_nodes"] = alg.get("avgNodesExplored", 0.0)
    return snap


def warm_up(base, sc, poll):
    """Fill the network to target before measuring, so the run doesn't include
    the ramp from 0 vehicles. Caps at 60 real seconds so an unreachable target
    doesn't hang forever."""
    api_get(base, "/api/spawnRate", {"value": "10"})
    target = sc["target"]
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline:
        snap = snapshot(api_get(base, "/api/state"))
        if snap["active"] >= target - 5:
            break
        time.sleep(poll)
    time.sleep(20)  # let the jam pattern settle


def run_one(base, sc, algo, sim_seconds, poll):
    api_get(base, "/api/clearVehicles")
    api_get(base, "/api/speed", {"x": str(SPEED)})
    api_get(base, "/api/auto", {"on": "1"})
    if not sc["closure"]:
        api_get(base, "/api/randomEvents", {"on": "0"})
    api_get(base, "/api/target", {"value": str(sc["target"])})
    if sc["closure"]:
        api_get(base, "/api/event", {"type": "closure"})
    api_get(base, "/api/algorithm", {"name": algo})

    warm_up(base, sc, poll)

    start = snapshot(api_get(base, "/api/state"), algo)
    t0 = start["time"]
    samples = []
    deadline = time.monotonic() + (sim_seconds / SPEED) * 3 + 30

    while True:
        state = api_get(base, "/api/state")
        snap = snapshot(state, algo)
        samples.append(snap["congestion"])
        if snap["time"] - t0 >= sim_seconds:
            break
        if time.monotonic() > deadline:
            print(f"  [warn] sim stalled at {snap['time'] - t0:.0f}/{sim_seconds}s",
                  file=sys.stderr)
            break
        time.sleep(poll)
    end = snapshot(state, algo)

    trips = end["trips"] - start["trips"]
    total0 = start["avg_travel"] * start["trips"]
    total1 = end["avg_travel"] * end["trips"]
    avg_travel = (total1 - total0) / trips if trips else 0.0

    algo_comps = end["algo_computations"] - start["algo_computations"]
    m0 = start["algo_avg_micros"] * start["algo_computations"]
    m1 = end["algo_avg_micros"] * end["algo_computations"]
    avg_micros = (m1 - m0) / algo_comps if algo_comps else 0.0
    n0 = start["algo_avg_nodes"] * start["algo_computations"]
    n1 = end["algo_avg_nodes"] * end["algo_computations"]
    avg_nodes = (n1 - n0) / algo_comps if algo_comps else 0.0

    recomputes = end["computations"] - start["computations"]
    sim_mins = (end["time"] - t0) / 60.0

    return {
        "scenario": sc["key"],
        "algorithm": algo,
        "sim_seconds": round(end["time"] - t0, 1),
        "completed_trips": trips,
        "trips_per_sim_min": round(trips / sim_mins, 2) if sim_mins else 0.0,
        "avg_travel_time_s": round(avg_travel, 2),
        "avg_congestion": round(sum(samples) / len(samples), 3) if samples else 0.0,
        "route_recomputations": recomputes,
        "recomputes_per_sim_min": round(recomputes / sim_mins, 2) if sim_mins else 0.0,
        "active_vehicles_end": end["active"],
        "avg_nodes_per_recompute": round(avg_nodes, 1),
        "avg_micros_per_recompute": round(avg_micros, 1),
    }


def aggregate(raw_rows):
    groups = {}
    for r in raw_rows:
        groups.setdefault((r["scenario"], r["algorithm"]), []).append(r)
    agg = []
    for (sc, algo), reps in groups.items():
        row = {"scenario": sc, "algorithm": algo, "repeats": len(reps)}
        for m in METRICS:
            vals = [r[m] for r in reps]
            row[m] = round(sum(vals) / len(vals), 3)
            row[m + "_range"] = round(max(vals) - min(vals), 3)
        agg.append(row)
    return agg


def write_csv(raw_rows, run_id, ts_header):
    path = f"docs/results_{run_id}.csv"
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["timestamp", "repeat", "scenario", "algorithm", "sim_seconds",
                    "completed_trips", "trips_per_sim_min", "avg_travel_time_s",
                    "avg_congestion", "route_recomputations", "recomputes_per_sim_min",
                    "active_vehicles_end", "avg_nodes_per_recompute",
                    "avg_micros_per_recompute"])
        for r in raw_rows:
            w.writerow([ts_header, r["repeat"], r["scenario"], r["algorithm"],
                        r["sim_seconds"], r["completed_trips"], r["trips_per_sim_min"],
                        r["avg_travel_time_s"], r["avg_congestion"],
                        r["route_recomputations"], r["recomputes_per_sim_min"],
                        r["active_vehicles_end"], r["avg_nodes_per_recompute"],
                        r["avg_micros_per_recompute"]])
    return path


def write_charts(agg_rows, run_id):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed -- skipping charts", file=sys.stderr)
        return False

    labels = [sc["label"] for sc in SCENARIOS]
    by = {sc["key"]: {} for sc in SCENARIOS}
    for r in agg_rows:
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


def fmt(val, rng, show_range):
    if show_range and rng > 0:
        return f"{val} ± {rng}"
    return str(val)


def write_report(agg_rows, charts_ok, run_id, ts_header, repeats):
    show_range = repeats > 1
    table_lines = [
        "| Scenario | Algorithm | Trips/min | Travel (s) | Congestion | Recomp/min | Active | Nodes/recomp | Micros/recomp |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    for sc in SCENARIOS:
        for r in sorted([x for x in agg_rows if x["scenario"] == sc["key"]],
                        key=lambda x: ALGORITHMS.index(x["algorithm"])):
            table_lines.append(
                f"| {sc['label']} | {r['algorithm']} | "
                f"{fmt(r['trips_per_sim_min'], r['trips_per_sim_min_range'], show_range)} | "
                f"{fmt(r['avg_travel_time_s'], r['avg_travel_time_s_range'], show_range)} | "
                f"{fmt(r['avg_congestion'], r['avg_congestion_range'], show_range)} | "
                f"{fmt(r['recomputes_per_sim_min'], r['recomputes_per_sim_min_range'], show_range)} | "
                f"{fmt(r['active_vehicles_end'], r['active_vehicles_end_range'], show_range)} | "
                f"{fmt(r['avg_nodes_per_recompute'], r['avg_nodes_per_recompute_range'], show_range)} | "
                f"{fmt(r['avg_micros_per_recompute'], r['avg_micros_per_recompute_range'], show_range)} |")
    table = "\n".join(table_lines)

    charts_section = ""
    if charts_ok:
        chart_block = "".join(
            f"![{title}](images/{run_id}/experiments_{stem}.png)\n\n"
            for _, title, _, stem in CHARTS)
        charts_section = f"### Charts\n\n{chart_block}"

    reps_note = f" ({repeats} repeats per combo, mean ± range)" if repeats > 1 else ""
    section = f"""## {ts_header}

Four scenarios x three algorithms{reps_note}; each run advanced {SIM_SECONDS} s of simulated time (x{SPEED} speed) after a warm-up. Closure picks a random road each run.

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
    p.add_argument("--sim-seconds", type=int, default=None)
    p.add_argument("--poll-interval", type=float, default=POLL_INTERVAL)
    p.add_argument("--repeats", type=int, default=3,
                   help="repeats per scenario x algorithm combo")
    p.add_argument("--smoke", action="store_true", help="quick 1-run check")
    args = p.parse_args()

    base = f"http://{args.host}:{args.port}"
    sim_seconds = args.sim_seconds or (60 if args.smoke else SIM_SECONDS)
    repeats = 1 if args.smoke else args.repeats

    if args.smoke:
        runs = [(SCENARIOS[0], "dijkstra", sim_seconds, 1)]
        print(f"[smoke] 1 run x {sim_seconds}s sim")
    else:
        runs = [(sc, a, sim_seconds, rep)
                for sc in SCENARIOS for a in ALGORITHMS
                for rep in range(1, repeats + 1)]
        print(f"{len(runs)} runs ({len(SCENARIOS)} x {len(ALGORITHMS)} x {repeats}), "
              f"~{len(runs) * sim_seconds / SPEED:.0f}s real at x{SPEED}.")

    try:
        api_get(base, "/api/state", timeout=5)
    except RuntimeError as e:
        print(str(e), file=sys.stderr)
        return 2

    raw_rows = []
    for i, (sc, algo, secs, rep) in enumerate(runs, 1):
        rep_str = f" ({rep}/{repeats})" if repeats > 1 else ""
        print(f"[{i}/{len(runs)}] {sc['key']} / {algo}{rep_str} ...", end=" ", flush=True)
        row = run_one(base, sc, algo, secs, args.poll_interval)
        row["repeat"] = rep
        raw_rows.append(row)
        print(f"trips/min={row['trips_per_sim_min']}  nodes={row['avg_nodes_per_recompute']}  "
              f"micros={row['avg_micros_per_recompute']}")

    now = datetime.now()
    run_id = now.strftime("%Y-%m-%d_%H-%M")
    ts_header = now.strftime("%Y-%m-%d %H:%M")

    csv_path = write_csv(raw_rows, run_id, ts_header)
    print(f"wrote {csv_path}")

    agg_rows = aggregate(raw_rows)
    charts_ok = write_charts(agg_rows, run_id)
    if charts_ok:
        print("wrote 5 charts in docs/")

    write_report(agg_rows, charts_ok, run_id, ts_header, repeats)
    print(f"appended to {REPORT_MD}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
