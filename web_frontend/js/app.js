// App entry: load the static map from the backend and draw it.
// (No vehicles yet — this is just the map-loading view.)

import { openSidebar, closeSidebar, prepareForm, handleAdd } from './menu.js';
import { add } from './features.js';
import { Camera } from './camera.js';
import { loadMap } from './api.js';
import { MapView } from './map_view.js';

// Expose the menu/sidebar handlers used by the injected HTML (onclick=...).
window.openSidebar = openSidebar;
window.closeSidebar = closeSidebar;
window.prepareForm = prepareForm;
window.handleAdd = handleAdd;
window.add = add;

const canvas = document.getElementById('mainCanvas');
const ctx = canvas.getContext('2d');
const camera = new Camera(canvas);

let mapView = null;      // set once the map JSON has loaded
let fitted = false;      // fit the camera once, after the first resize+load

function resize() {
    canvas.width = window.innerWidth;
    canvas.height = window.innerHeight;
    if (mapView && !fitted) { mapView.fit(camera, canvas); fitted = true; }
    camera.dirty = true;
}
window.addEventListener('resize', resize);
resize();

// Render only when the view changed. A city-scale map is tens of thousands of
// shapes, so redrawing every frame would be needlessly heavy.
function animate() {
    if (mapView && camera.dirty) {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        camera.begin(ctx);
        mapView.draw(ctx);
        camera.end(ctx);
        camera.dirty = false;
    }
    requestAnimationFrame(animate);
}

// Load the map, then start drawing.
async function start() {
    try {
        const data = await loadMap();
        mapView = new MapView(data);
        mapView.fit(camera, canvas);
        fitted = true;
        camera.dirty = true;
        console.log(`Map loaded: ${data.intersections.length} intersections, ` +
                    `${data.roads.length} roads, ${(data.buildings || []).length} buildings.`);
    } catch (err) {
        console.error(err);
        // Show the reason on the canvas so it's obvious the fetch failed.
        ctx.fillStyle = '#f87171';
        ctx.font = '16px Inter, sans-serif';
        ctx.fillText(String(err.message || err), 24, 40);
    }
}

start();
animate();
