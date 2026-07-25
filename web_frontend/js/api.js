// Backend data access for the web UI.
//
// Prefers the live server (GET /api/map) and falls back to the static
// web_frontend/map.json, so the UI works both ways:
//   - served by traffic_simulator   -> /api/map
//   - opened via a plain file server -> ../map.json
// Later PRs add loadState() here for /api/state without touching the renderer.

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
