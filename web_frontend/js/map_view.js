// Static map renderer.
//
// Takes the JSON from api.loadMap() and draws it in world coordinates (meters).
// The Camera applies pan/zoom, so everything here is drawn at true map scale.
// Draw order (back to front): zones -> roads -> buildings -> intersections.

const COLORS = {
    ground: "#1f2937",
    zone: "rgba(255,255,255,0.04)",
    road: "#4b5563",
    roadEdge: "#374151",
    laneMark: "rgba(255,255,255,0.5)",
    building: "#5b6472",
    buildingEdge: "#2b313b",
    node: "#9ca3af",
    nodeSignal: "#f59e0b",
};

export class MapView {
    constructor(mapData) {
        this.map = mapData;
        // Fast id -> intersection lookup so roads can find their endpoints.
        this.nodeById = {};
        for (const n of mapData.intersections) this.nodeById[n.id] = n;
        this.bounds = this._computeBounds();
    }

    _computeBounds() {
        let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
        const grow = (x, y) => {
            if (x < minX) minX = x; if (x > maxX) maxX = x;
            if (y < minY) minY = y; if (y > maxY) maxY = y;
        };
        for (const n of this.map.intersections) grow(n.x, n.y);
        for (const b of this.map.buildings || []) {
            for (const [x, y] of b.footprint || []) grow(x, y);
        }
        if (!isFinite(minX)) { minX = minY = 0; maxX = maxY = 1000; }
        return { minX, minY, maxX, maxY,
                 w: Math.max(1, maxX - minX), h: Math.max(1, maxY - minY) };
    }

    // Position the camera so the whole map fits the canvas with a small margin.
    fit(camera, canvas) {
        const b = this.bounds;
        const zoom = 0.9 * Math.min(canvas.width / b.w, canvas.height / b.h);
        const cx = (b.minX + b.maxX) / 2;
        const cy = (b.minY + b.maxY) / 2;
        camera.zoom = zoom;
        camera.x = canvas.width / 2 - cx * zoom;
        camera.y = canvas.height / 2 - cy * zoom;
    }

    draw(ctx) {
        this._drawZones(ctx);
        this._drawRoads(ctx);
        this._drawBuildings(ctx);
        this._drawNodes(ctx);
    }

    _drawZones(ctx) {
        ctx.fillStyle = COLORS.zone;
        for (const z of this.map.zones || []) {
            ctx.fillRect(z.x, z.y, z.w, z.h);
        }
    }

    _drawRoads(ctx) {
        ctx.lineCap = "round";
        ctx.lineJoin = "round";
        ctx.strokeStyle = COLORS.road;
        // City-scale maps have tens of thousands of segments. Batch all roads of
        // the same width into one path so we issue a handful of stroke() calls
        // instead of one per road.
        const byWidth = new Map();
        for (const r of this.map.roads) {
            const a = this.nodeById[r.from], b = this.nodeById[r.to];
            if (!a || !b) continue;                    // dangling reference: skip
            const width = 6 + (r.lanes || 1) * 6;      // world-unit road width
            let path = byWidth.get(width);
            if (!path) { path = new Path2D(); byWidth.set(width, path); }
            path.moveTo(a.x, a.y);
            path.lineTo(b.x, b.y);
        }
        for (const [width, path] of byWidth) {
            ctx.lineWidth = width;
            ctx.stroke(path);
        }
    }

    _drawBuildings(ctx) {
        // Batch every footprint into one path, then a single fill + stroke.
        const path = new Path2D();
        for (const bld of this.map.buildings || []) {
            const fp = bld.footprint || [];
            if (fp.length < 3) continue;
            path.moveTo(fp[0][0], fp[0][1]);
            for (let i = 1; i < fp.length; i++) path.lineTo(fp[i][0], fp[i][1]);
            path.closePath();
        }
        ctx.fillStyle = COLORS.building; ctx.fill(path);
        ctx.lineWidth = 1; ctx.strokeStyle = COLORS.buildingEdge; ctx.stroke(path);
    }

    _drawNodes(ctx) {
        // Only real junctions get a dot. Waypoints (zone="waypoint") are shape
        // points that make roads follow curves — drawing them would bury the map
        // in dots, so we skip them (same as the reference).
        // Batch into two paths (plain vs signalized) for a couple of fill calls.
        const R = 12;
        const plain = new Path2D(), signal = new Path2D();
        for (const n of this.map.intersections) {
            if (n.zone === "waypoint") continue;
            const p = n.signal ? signal : plain;
            p.moveTo(n.x + R, n.y);
            p.arc(n.x, n.y, R, 0, Math.PI * 2);
        }
        ctx.fillStyle = COLORS.node; ctx.fill(plain);
        ctx.fillStyle = COLORS.nodeSignal; ctx.fill(signal);
    }
}
