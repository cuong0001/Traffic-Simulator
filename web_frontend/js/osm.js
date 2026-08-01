// ---------------------------------------------------------------------------
// osm.js — pick an area out of OpenStreetMap and import it as the simulation map.
//
// The whole download happens HERE, in the browser, not in the C++ backend. Two
// reasons, and the second is the real one:
//
//   1. Overpass is HTTPS. This project has no dependencies at all -- no libcurl,
//      no OpenSSL -- and adding a TLS stack to fetch one URL would be a large
//      price for one feature. The browser already has one.
//   2. The backend already accepts GeoJSON. MapLoader detects lon/lat input and
//      projects it itself, so if this module emits an ordinary OSM-style
//      GeoJSON document, every line of the loader works unchanged.
//
// So: raster tiles for the picker, Overpass for the data, a conversion to
// GeoJSON here, and a POST to /api/importMap.
// ---------------------------------------------------------------------------

const TILE = 256;
// Overpass instances, tried in order. More than one because a single instance
// goes down, rate-limits, or refuses a client often enough that hard-coding one
// makes the feature look broken when it is only busy.
const OVERPASS_MIRRORS = [
  "https://overpass-api.de/api/interpreter",
  "https://overpass.kumi.systems/api/interpreter",
  "https://overpass.private.coffee/api/interpreter",
];

// Web Mercator, the projection every raster tile server uses.
const lonToX = (lon, z) => ((lon + 180) / 360) * Math.pow(2, z);
const latToY = (lat, z) => {
  const r = (lat * Math.PI) / 180;
  return ((1 - Math.log(Math.tan(r) + 1 / Math.cos(r)) / Math.PI) / 2) * Math.pow(2, z);
};
const xToLon = (x, z) => (x / Math.pow(2, z)) * 360 - 180;
const yToLat = (y, z) => {
  const n = Math.PI - (2 * Math.PI * y) / Math.pow(2, z);
  return (180 / Math.PI) * Math.atan(0.5 * (Math.exp(n) - Math.exp(-n)));
};

// Rough ground distance, good enough for showing the user how big their
// selection is before they commit to downloading it.
function bboxSpanKm(b) {
  const midLat = ((b.north + b.south) / 2) * (Math.PI / 180);
  return {
    w: (b.east - b.west) * 111.32 * Math.cos(midLat),
    h: (b.north - b.south) * 110.57,
  };
}

export class OsmPicker {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.z = 14;
    this.centre = { lat: 10.7769, lon: 106.7009 };   // Ho Chi Minh City
    this.tiles = new Map();        // "z/x/y" -> Image
    this.sel = null;               // { north, south, east, west }
    this.drag = null;
    this.onChange = null;          // called whenever the selection changes
    this.active = false;
    this._bind();
  }

  // --- viewport helpers ---------------------------------------------------
  get w() { return this.canvas.width; }
  get h() { return this.canvas.height; }

  // Pixel position, in world tile-pixels at the current zoom, of the top-left
  // corner of the viewport.
  get origin() {
    return {
      x: lonToX(this.centre.lon, this.z) * TILE - this.w / 2,
      y: latToY(this.centre.lat, this.z) * TILE - this.h / 2,
    };
  }

  screenOf(lat, lon) {
    const o = this.origin;
    return { x: lonToX(lon, this.z) * TILE - o.x, y: latToY(lat, this.z) * TILE - o.y };
  }

  latLonOf(px, py) {
    const o = this.origin;
    return {
      lat: yToLat((o.y + py) / TILE, this.z),
      lon: xToLon((o.x + px) / TILE, this.z),
    };
  }

  // --- input ---------------------------------------------------------------
  _bind() {
    const pos = (e) => {
      const r = this.canvas.getBoundingClientRect();
      return {
        x: (e.clientX - r.left) * (this.canvas.width / this.canvas.clientWidth),
        y: (e.clientY - r.top) * (this.canvas.height / this.canvas.clientHeight),
      };
    };

    this.canvas.addEventListener("pointerdown", (e) => {
      if (!this.active) return;
      const p = pos(e);
      // Shift-drag pans, plain drag selects. Selecting is the common action, so
      // it gets the plain gesture.
      this.drag = { mode: e.shiftKey ? "pan" : "select", from: p, to: p,
                    centre0: { ...this.centre } };
      this.canvas.setPointerCapture(e.pointerId);
    });

    this.canvas.addEventListener("pointermove", (e) => {
      if (!this.active || !this.drag) return;
      const p = pos(e);
      if (this.drag.mode === "pan") {
        const o0 = {
          x: lonToX(this.drag.centre0.lon, this.z) * TILE,
          y: latToY(this.drag.centre0.lat, this.z) * TILE,
        };
        const nx = o0.x - (p.x - this.drag.from.x);
        const ny = o0.y - (p.y - this.drag.from.y);
        this.centre = { lat: yToLat(ny / TILE, this.z), lon: xToLon(nx / TILE, this.z) };
      } else {
        this.drag.to = p;
        this._setSelectionFromDrag();
      }
      this.draw();
    });

    const end = (e) => {
      if (!this.drag) return;
      this.drag = null;
      if (e.pointerId != null && this.canvas.hasPointerCapture?.(e.pointerId))
        this.canvas.releasePointerCapture(e.pointerId);
      this.draw();
    };
    this.canvas.addEventListener("pointerup", end);
    this.canvas.addEventListener("pointercancel", end);

    this.canvas.addEventListener("wheel", (e) => {
      if (!this.active) return;
      e.preventDefault();
      const p = pos(e);
      const under = this.latLonOf(p.x, p.y);      // keep this point fixed
      const nz = Math.max(10, Math.min(18, this.z + (e.deltaY > 0 ? -1 : 1)));
      if (nz === this.z) return;
      this.z = nz;
      const after = this.screenOf(under.lat, under.lon);
      const drift = this.latLonOf(this.w / 2 + (after.x - p.x), this.h / 2 + (after.y - p.y));
      this.centre = drift;
      this.draw();
    }, { passive: false });
  }

  _setSelectionFromDrag() {
    const a = this.latLonOf(this.drag.from.x, this.drag.from.y);
    const b = this.latLonOf(this.drag.to.x, this.drag.to.y);
    this.sel = {
      north: Math.max(a.lat, b.lat), south: Math.min(a.lat, b.lat),
      east: Math.max(a.lon, b.lon), west: Math.min(a.lon, b.lon),
    };
    this.onChange?.(this.sel, bboxSpanKm(this.sel));
  }

  clearSelection() {
    this.sel = null;
    this.onChange?.(null, null);
    this.draw();
  }

  // --- drawing --------------------------------------------------------------
  _tile(z, x, y) {
    const key = `${z}/${x}/${y}`;
    let img = this.tiles.get(key);
    if (img) return img;
    img = new Image();
    img.crossOrigin = "anonymous";
    // Tiles come straight from openstreetmap.org. Their usage policy asks for
    // a real referrer and no bulk scraping; a hand-panned picker is well inside
    // it, but this is why the zoom is capped at 18 and the app does not prefetch.
    img.src = `https://tile.openstreetmap.org/${z}/${x}/${y}.png`;
    img.onload = () => this.draw();
    this.tiles.set(key, img);
    return img;
  }

  draw() {
    if (!this.active) return;
    const g = this.ctx;
    const o = this.origin;
    g.fillStyle = "#dfe6e9";
    g.fillRect(0, 0, this.w, this.h);

    const n = Math.pow(2, this.z);
    const x0 = Math.floor(o.x / TILE), y0 = Math.floor(o.y / TILE);
    const x1 = Math.floor((o.x + this.w) / TILE), y1 = Math.floor((o.y + this.h) / TILE);
    for (let x = x0; x <= x1; x++) {
      for (let y = y0; y <= y1; y++) {
        if (y < 0 || y >= n) continue;
        const wrapped = ((x % n) + n) % n;          // the world repeats east-west
        const img = this._tile(this.z, wrapped, y);
        if (img.complete && img.naturalWidth) {
          g.drawImage(img, x * TILE - o.x, y * TILE - o.y, TILE, TILE);
        }
      }
    }

    if (this.sel) {
      const a = this.screenOf(this.sel.north, this.sel.west);
      const b = this.screenOf(this.sel.south, this.sel.east);
      g.fillStyle = "rgba(34,197,94,.18)";
      g.fillRect(a.x, a.y, b.x - a.x, b.y - a.y);
      g.strokeStyle = "#16a34a";
      g.lineWidth = 2;
      g.setLineDash([7, 5]);
      g.strokeRect(a.x, a.y, b.x - a.x, b.y - a.y);
      g.setLineDash([]);
    }

    // Attribution is a licence condition of using OSM tiles, not decoration.
    const label = "© OpenStreetMap contributors";
    g.font = "12px system-ui, sans-serif";
    const tw = g.measureText(label).width + 10;
    g.fillStyle = "rgba(255,255,255,.75)";
    g.fillRect(this.w - tw - 4, this.h - 22, tw, 18);
    g.fillStyle = "#333";
    g.textAlign = "left";
    g.textBaseline = "middle";
    g.fillText(label, this.w - tw + 1, this.h - 13);
  }
}

// ---------------------------------------------------------------------------
// Overpass -> GeoJSON
// ---------------------------------------------------------------------------

// Only the road classes a car can actually drive. Without this filter an
// extract is mostly footpaths and the simulation spawns vehicles onto pavements.
const DRIVABLE =
  "motorway|trunk|primary|secondary|tertiary|unclassified|residential|" +
  "living_street|service|motorway_link|trunk_link|primary_link|secondary_link|tertiary_link";

export function overpassQuery(b) {
  const bbox = `${b.south},${b.west},${b.north},${b.east}`;
  // `way(...)["highway"~"..."]` then `>;` to pull in every node the ways
  // reference -- without the recurse-down the geometry would be missing.
  return `[out:json][timeout:90];
(
  way["highway"~"^(${DRIVABLE})$"](${bbox});
);
(._;>;);
out body;`;
}

export async function fetchOverpass(bbox, onProgress) {
  // The query goes as `data=<urlencoded>` in a form-encoded body. That is the
  // documented form; posting the raw query as text/plain is rejected outright
  // by some instances.
  const payload = "data=" + encodeURIComponent(overpassQuery(bbox));
  let lastError = "";

  for (let i = 0; i < OVERPASS_MIRRORS.length; i++) {
    const url = OVERPASS_MIRRORS[i];
    onProgress?.(i === 0
      ? "Asking OpenStreetMap for the roads in this area…"
      : `Mirror ${i} was busy — trying another…`);
    let res;
    try {
      res = await fetch(url, {
        method: "POST",
        headers: { "Content-Type": "application/x-www-form-urlencoded" },
        body: payload,
      });
    } catch (e) {
      lastError = "network error";
      continue;                       // instance unreachable: try the next
    }
    if (res.ok) {
      onProgress?.("Converting to a map…");
      return res.json();
    }
    // 429 (too many requests) and 504 (gateway timeout) mean "busy, try again",
    // which is common enough that another mirror usually answers straight away.
    lastError = res.status === 429 ? "rate-limited"
              : res.status === 504 ? "timed out"
              : `HTTP ${res.status}`;
  }

  throw new Error(
    `Every OpenStreetMap server refused the request (${lastError}). ` +
    `They throttle by IP, so waiting a minute usually clears it.`);
}

// Convert an Overpass JSON response into the GeoJSON dialect MapLoader already
// reads: LineString features whose properties carry the OSM tags.
export function overpassToGeoJson(osm) {
  const nodes = new Map();
  const features = [];
  for (const el of osm.elements || []) {
    if (el.type !== "node") continue;
    nodes.set(el.id, [el.lon, el.lat]);
    // Traffic signals ride along in the reply already: they are vertices of the
    // ways we asked for, and `>` pulls them in with their tags. MapLoader reads
    // them from Point features, so emitting them here is the difference between
    // an imported city that has working lights and one that has none.
    if (el.tags && el.tags.highway === "traffic_signals") {
      features.push({
        type: "Feature",
        properties: { "@id": `node/${el.id}`, highway: "traffic_signals" },
        geometry: { type: "Point", coordinates: [el.lon, el.lat] },
      });
    }
  }

  for (const el of osm.elements || []) {
    if (el.type !== "way" || !el.nodes || el.nodes.length < 2) continue;
    const coords = [];
    for (const id of el.nodes) {
      const c = nodes.get(id);
      if (c) coords.push(c);
    }
    if (coords.length < 2) continue;      // way clipped by the bbox edge

    const t = el.tags || {};
    const props = { "@id": `way/${el.id}` };
    // Copy through only what MapLoader looks at. Passing every OSM tag would
    // work but bloats the saved file for no gain.
    for (const k of ["highway", "oneway", "lanes", "maxspeed", "name", "junction"]) {
      if (t[k] !== undefined) props[k] = t[k];
    }
    features.push({ type: "Feature", properties: props,
                    geometry: { type: "LineString", coordinates: coords } });
  }

  // Deliberately NOT stamped with `generator: "urban-traffic-simulator"`.
  // MapLoader uses that stamp to decide the document is in its OWN format, and
  // then looks for native node features that an OSM extract does not have --
  // the import fails with "contains no nodes (empty graph)". This is an OSM
  // document and must be sniffed as one.
  return { type: "FeatureCollection", features };
}

export async function importArea(bbox, name, onProgress) {
  const span = bboxSpanKm(bbox);
  if (span.w * span.h > 25) {
    throw new Error(
      `That area is ${span.w.toFixed(1)} x ${span.h.toFixed(1)} km. ` +
      `Pick something under about 5x5 km — a bigger extract takes minutes to ` +
      `download and makes a graph too large to simulate smoothly.`);
  }

  const osm = await fetchOverpass(bbox, onProgress);
  const geo = overpassToGeoJson(osm);
  const roads = geo.features.filter((f) => f.geometry.type === "LineString").length;
  if (!roads) {
    throw new Error("No drivable roads in that area — try somewhere with streets on it");
  }

  onProgress?.(`Sending ${roads} roads to the simulator…`);
  const res = await fetch(`/api/importMap?name=${encodeURIComponent(name)}`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(geo),
  });
  const out = await res.json();
  if (!out.ok) throw new Error(out.error || "the simulator refused the map");
  return { roads, signals: geo.features.length - roads, warning: out.warning || "" };
}
