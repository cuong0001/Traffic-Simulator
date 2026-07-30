// Backend data access for the web UI.
//
// Prefers the live server (GET /api/map) and falls back to the static
// web_frontend/map.json, so the UI works both ways:
//   - served by traffic_simulator   -> /api/map
//   - opened via a plain file server -> ../map.json
//
// The simulation endpoints (/api/state and the controls) only exist while the
// C++ server runs; every helper below fails soft and returns null, so the
// static map keeps rendering when the backend is not reachable.

export async function loadMap() {
    // 1. live server endpoint (same origin)
    try {
        const res = await fetch("/api/map", { cache: "no-store" });
        if (res.ok) return await res.json();
    } catch (_) { /* not served by our C++ server; try the file */ }

    // 2. static file next to the page (html/ -> ../map.json)
    const res = await fetch("../map.json", { cache: "no-store" });
    if (!res.ok) {
        throw new Error(
            `Could not load the map from /api/map or ../map.json (${res.status}). ` +
            `Run: traffic_simulator <map.geojson> [port]`);
    }
    return res.json();  // { intersections, roads, buildings, zones }
}

// One simulation snapshot:
//   { time, paused, speed, vehicles[{id,road,pos,speed,type}],
//     lights[{id,state}], roads[{id,congestion,closed}], events[], stats{} }
// Returns null when the simulation server is not there.
export async function loadState() {
    try {
        const res = await fetch("/api/state", { cache: "no-store" });
        if (!res.ok) return null;
        return await res.json();
    } catch (_) {
        return null;
    }
}

// Control call. Never throws: a failed control must not break the render loop.
async function control(path, params = {}) {
    const qs = new URLSearchParams(params).toString();
    try {
        const res = await fetch(qs ? `${path}?${qs}` : path, { cache: "no-store" });
        if (!res.ok) return null;
        return await res.json();
    } catch (_) {
        return null;
    }
}

export const Sim = {
    // playback
    pause:    ()                   => control("/api/pause"),
    resume:   ()                   => control("/api/resume"),
    setSpeed: (x)                  => control("/api/speed", { x }),

    // traffic generation
    setAuto:      (on)    => control("/api/auto", { on: on ? 1 : 0 }),
    setTarget:    (value) => control("/api/target", { value }),
    setSpawnRate: (value) => control("/api/spawnRate", { value }),
    setAlgorithm: (name)  => control("/api/algorithm", { name }),

    // vehicles
    spawn:        (from, to, type) => control("/api/spawn", { from, to, type }),
    event:        (type, target, dur) => control("/api/event", { type, target, dur }),
    vehicleDetail:(id)    => control("/api/vehicle/detail", { id }),
    removeVehicle:(id)    => control("/api/removeVehicle", { id }),
    clearVehicles:()      => control("/api/clearVehicles"),
    updateVehicle:(id, opts) => control("/api/vehicle", { id, ...opts }),

    // live map editing. addRoad endpoints are {kind:"junction"|"road"|"free"}:
    // "road" splits the road under (x,y), "free" creates a junction there.
    addRoad: (from, to, opts = {}) => control("/api/addRoad", {
        fromKind: from.kind, fromX: from.x ?? "", fromY: from.y ?? "", from: from.id ?? "",
        toKind: to.kind,     toX: to.x ?? "",     toY: to.y ?? "",     to: to.id ?? "",
        speed: opts.speed ?? 40, lanes: opts.lanes ?? 1,
        oneway: opts.oneway ? "true" : "false",
    }),
    removeRoad:         (id) => control("/api/removeRoad", { id }),
    removeIntersection: (id) => control("/api/removeIntersection", { id }),
    updateRoad: (id, speed, closed) =>
        control("/api/updateRoad", { id, speed, closed: closed ? "true" : "false" }),
    setLight: (id, on, green) =>
        control("/api/setLight", { id, on: on ? "true" : "false", green }),
    addBuilding: (x, y, kind, name, size) =>
        control("/api/addBuilding", { x, y, kind, name, size }),
    removeBuilding: (id) => control("/api/removeBuilding", { id }),

    // map files
    maps:    ()     => control("/api/maps"),
    saveMap: (name) => control("/api/saveMap", { name }),
    loadMap: (name) => control("/api/loadMap", { name }),
};
