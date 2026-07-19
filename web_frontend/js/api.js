// Backend data access for the web UI.
//
// For now the backend (traffic_simulator) writes the static map to
// web_frontend/map.json. We fetch it once and draw it. Later this same module
// can point at a live HTTP endpoint (e.g. /api/map, /api/state) without the
// rest of the UI changing.

// Fetch the static map. Path is relative to the HTML document (html/index.html),
// so "../map.json" resolves to web_frontend/map.json.
export async function loadMap(url = "../map.json") {
    const res = await fetch(url);
    if (!res.ok) {
        throw new Error(`Could not load map (${res.status}) from ${url}. ` +
            `Run traffic_simulator first to generate map.json.`);
    }
    return res.json();  // { intersections, roads, buildings, zones }
}
