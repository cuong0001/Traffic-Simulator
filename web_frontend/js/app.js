const canvas = document.getElementById('simCanvas');
const ctx = canvas.getContext('2d', { alpha: false });

let camera = { x: 50, y: 50, zoom: 0.5 };
let isDragging = false;
let dragStart = { x: 0, y: 0 };
const PIXELS_PER_METER = 5.0; 

// Data containers
let nodes = {};
let edges = [];
let buildings = [];

function resizeCanvas() {
    canvas.width = canvas.parentElement.clientWidth;
    canvas.height = canvas.parentElement.clientHeight;
    render();
}
window.addEventListener('resize', resizeCanvas);
resizeCanvas();

// Input Handling
canvas.addEventListener('mousedown', e => {
    isDragging = true;
    dragStart = { x: e.clientX, y: e.clientY };
});
canvas.addEventListener('mousemove', e => {
    if (!isDragging) return;
    camera.x += e.clientX - dragStart.x;
    camera.y += e.clientY - dragStart.y;
    dragStart = { x: e.clientX, y: e.clientY };
    render();
});
window.addEventListener('mouseup', () => isDragging = false);

canvas.addEventListener('wheel', e => {
    const zoomFactor = 1.1;
    const oldZoom = camera.zoom;
    camera.zoom = e.deltaY < 0 ? camera.zoom * zoomFactor : camera.zoom / zoomFactor;
    
    const mouseX = e.clientX - canvas.getBoundingClientRect().left;
    const mouseY = e.clientY - canvas.getBoundingClientRect().top;
    
    camera.x = mouseX - (mouseX - camera.x) * (camera.zoom / oldZoom);
    camera.y = mouseY - (mouseY - camera.y) * (camera.zoom / oldZoom);
    render();
});

// CSV Parser
function parseCSV(text) {
    const lines = text.split('\n').map(l => l.trim()).filter(l => l.length > 0);
    const headers = lines[0].split(',').map(h => h.trim());
    const result = [];
    for (let i = 1; i < lines.length; i++) {
        const values = lines[i].split(',').map(v => v.trim());
        let obj = {};
        for (let j = 0; j < headers.length; j++) {
            obj[headers[j]] = values[j];
        }
        result.push(obj);
    }
    return result;
}

// Map Loader UI
const loaderPanel = document.getElementById('map-loader');
const loaderError = document.getElementById('loader-error');
const loadingAlert = document.getElementById('loading-alert');
const openLoaderBtn = document.getElementById('btn-open-loader');
const fileInput = document.getElementById('csv-file-input');

function showLoaderError(message) {
    loaderError.textContent = message;
    loaderError.classList.remove('hidden');
}

function clearLoaderError() {
    loaderError.textContent = '';
    loaderError.classList.add('hidden');
}

function onMapLoaded() {
    loadingAlert.classList.add('hidden');
    loaderPanel.classList.add('hidden');
    openLoaderBtn.classList.remove('hidden');
}

// Data Loading
function resetMapState() {
    nodes = {};
    edges = [];
    buildings = [];
}

// Parses the 3 CSV texts, replaces the current map, fits the camera.
// Throws on fatally invalid data (no usable nodes).
function buildMap(nodesText, roadsText, buildingsText) {
    resetMapState();

    // Process Nodes
    const nodesArr = parseCSV(nodesText);
    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;

    nodesArr.forEach(n => {
        const x = parseFloat(n.x);
        const y = parseFloat(n.y);
        if (!n.id || isNaN(x) || isNaN(y)) {
            console.warn('Skipping invalid node row:', n);
            return;
        }
        nodes[n.id] = { id: n.id, x, y };
        minX = Math.min(minX, x); maxX = Math.max(maxX, x);
        minY = Math.min(minY, y); maxY = Math.max(maxY, y);
    });

    if (Object.keys(nodes).length === 0) {
        throw new Error('No valid nodes found in nodes CSV.');
    }

    // Process Edges
    const edgesArr = parseCSV(roadsText);
    edgesArr.forEach(e => {
        if (!nodes[e.source] || !nodes[e.target]) {
            console.warn('Skipping road with unknown node reference:', e);
            return;
        }
        edges.push({
            id: e.id,
            source: nodes[e.source],
            target: nodes[e.target],
            lanes: parseInt(e.lanes) || 1,
            is_one_way: e.is_one_way === '1'
        });
    });

    // Process Buildings
    const bArr = parseCSV(buildingsText);
    bArr.forEach(b => {
        if (!nodes[b.node_id]) {
            console.warn('Skipping building with unknown node reference:', b);
            return;
        }
        buildings.push({
            name: b.name,
            node_id: b.node_id,
            type: b.type
        });
    });

    // Auto-fit camera
    const margin = 80;
    const zoom = Math.min(
        (canvas.width  - margin * 2) / (maxX - minX || 1),
        (canvas.height - margin * 2) / (maxY - minY || 1),
        1.5
    );
    camera.zoom = zoom;
    camera.x = (canvas.width  - (maxX - minX) * zoom) / 2 - minX * zoom;
    camera.y = (canvas.height - (maxY - minY) * zoom) / 2 - minY * zoom;

    onMapLoaded();
    render();
}

async function loadDefaultMap() {
    clearLoaderError();
    loadingAlert.classList.remove('hidden');
    try {
        const [nodesRes, roadsRes, buildingsRes] = await Promise.all([
            fetch('../../data/maps/nodes.csv'),
            fetch('../../data/maps/roads.csv'),
            fetch('../../data/maps/buildings.csv')
        ]);

        if (!nodesRes.ok || !roadsRes.ok || !buildingsRes.ok) {
            throw new Error("Could not load map data. Are you running a local web server?");
        }

        const nodesText = await nodesRes.text();
        const roadsText = await roadsRes.text();
        const buildingsText = await buildingsRes.text();

        buildMap(nodesText, roadsText, buildingsText);
    } catch (err) {
        console.error(err);
        loadingAlert.classList.add('hidden');
        showLoaderError(`Error loading default map: ${err.message}`);
    }
}

// Identifies a CSV's role (nodes/roads/buildings) from its header line.
function identifyCsvRole(text) {
    const header = text.split('\n')[0].toLowerCase().split(',').map(h => h.trim());
    if (header.includes('source') && header.includes('target')) return 'roads';
    if (header.includes('node_id')) return 'buildings';
    if (header.includes('x') && header.includes('y')) return 'nodes';
    return null;
}

async function loadCustomMap(fileList) {
    clearLoaderError();
    const files = Array.from(fileList);
    if (files.length !== 3) {
        showLoaderError('Please select exactly 3 CSV files (nodes, roads, buildings).');
        return;
    }

    loadingAlert.classList.remove('hidden');
    try {
        const texts = {};
        for (const file of files) {
            const text = await file.text();
            let role = identifyCsvRole(text);
            if (!role) {
                // Fallback: guess from filename
                const name = file.name.toLowerCase();
                if (name.includes('node')) role = 'nodes';
                else if (name.includes('road')) role = 'roads';
                else if (name.includes('building')) role = 'buildings';
            }
            if (!role) {
                throw new Error(`Could not identify "${file.name}". Expected headers:\nnodes: id,x,y\nroads: id,source,target,lanes,speed_limit,is_one_way\nbuildings: name,node_id,type`);
            }
            if (texts[role]) {
                throw new Error(`Both "${file.name}" and another file look like the ${role} CSV.`);
            }
            texts[role] = text;
        }

        const missing = ['nodes', 'roads', 'buildings'].filter(r => !texts[r]);
        if (missing.length > 0) {
            throw new Error(`Missing CSV file(s): ${missing.join(', ')}.`);
        }

        buildMap(texts.nodes, texts.roads, texts.buildings);
    } catch (err) {
        console.error(err);
        loadingAlert.classList.add('hidden');
        showLoaderError(err.message);
    }
}

// Rendering
function render() {
    ctx.fillStyle = '#24242b';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    
    // 1. Draw Edges (Roads)
    edges.forEach(edge => {
        if (!edge.source || !edge.target) return;
        
        const sx = edge.source.x * camera.zoom + camera.x;
        const sy = edge.source.y * camera.zoom + camera.y;
        const tx = edge.target.x * camera.zoom + camera.x;
        const ty = edge.target.y * camera.zoom + camera.y;
        
        const dx = tx - sx;
        const dy = ty - sy;
        const len = Math.hypot(dx, dy);
        
        let shiftX = 0, shiftY = 0;
        if (!edge.is_one_way && len > 0) {
            // Shift to the right (assuming right-hand driving)
            const nx = dy / len;
            const ny = -dx / len;
            const shiftPx = (edge.lanes * 3.5 / 2.0) * PIXELS_PER_METER * camera.zoom;
            shiftX = nx * shiftPx;
            shiftY = ny * shiftPx;
        }
        
        ctx.beginPath();
        ctx.moveTo(sx + shiftX, sy + shiftY);
        ctx.lineTo(tx + shiftX, ty + shiftY);
        
        ctx.strokeStyle = '#3b3b44';
        ctx.lineWidth = edge.lanes * 3.5 * PIXELS_PER_METER * camera.zoom;
        ctx.stroke();
        
        // Draw lane markings
        if (edge.lanes > 1) {
            ctx.strokeStyle = 'rgba(255,255,255,0.4)';
            ctx.lineWidth = 1.5 * camera.zoom;
            ctx.setLineDash([15 * camera.zoom, 15 * camera.zoom]);
            
            for(let j=1; j<edge.lanes; j++){
                let laneOffset = (j - edge.lanes / 2.0) * 3.5 * PIXELS_PER_METER * camera.zoom;
                
                const nxOffset = len > 0 ? -dy / len : 0;
                const nyOffset = len > 0 ? dx / len : 0;
                
                ctx.beginPath();
                ctx.moveTo(sx + shiftX + nxOffset * laneOffset, sy + shiftY + nyOffset * laneOffset);
                ctx.lineTo(tx + shiftX + nxOffset * laneOffset, ty + shiftY + nyOffset * laneOffset);
                ctx.stroke();
            }
            ctx.setLineDash([]);
        }
    });

    // 2. Draw Nodes (Intersections)
    Object.values(nodes).forEach(n => {
        const nx = n.x * camera.zoom + camera.x;
        const ny = n.y * camera.zoom + camera.y;
        
        ctx.beginPath();
        ctx.arc(nx, ny, 10 * camera.zoom, 0, Math.PI * 2);
        ctx.fillStyle = '#666';
        ctx.fill();
    });

    // 3. Draw Buildings
    buildings.forEach(b => {
        const node = nodes[b.node_id];
        if (!node) return;
        
        const bx = node.x * camera.zoom + camera.x;
        const by = node.y * camera.zoom + camera.y;
        const size = 60 * camera.zoom;
        
        ctx.fillStyle = b.type === 'hospital' ? '#ff4a4a' : b.type === 'school' ? '#ffb84d' : '#4f8aff';
        ctx.fillRect(bx - size/2, by - size/2, size, size);
        
        ctx.fillStyle = 'white';
        ctx.font = `bold ${14 * camera.zoom}px Inter`;
        ctx.textAlign = 'center';
        ctx.fillText(b.name, bx, by - size/2 - 10);
    });
}

// Wire up loader controls
document.getElementById('btn-default-map').addEventListener('click', loadDefaultMap);
document.getElementById('btn-custom-map').addEventListener('click', () => fileInput.click());
fileInput.addEventListener('change', e => {
    loadCustomMap(e.target.files);
    e.target.value = ''; // allow re-selecting the same files
});
openLoaderBtn.addEventListener('click', () => {
    clearLoaderError();
    loaderPanel.classList.remove('hidden');
});
