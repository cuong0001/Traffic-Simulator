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

// Data Loading
async function loadMapData() {
    try {
        const [nodesRes, roadsRes, buildingsRes] = await Promise.all([
            fetch('../../data/maps/nodes.csv'),
            fetch('../../data/maps/roads.csv'),
            fetch('../../data/maps/buildings.csv')
        ]);
        
        if (!nodesRes.ok) throw new Error("Could not load map data. Are you running a local web server?");
        
        const nodesText = await nodesRes.text();
        const roadsText = await roadsRes.text();
        const buildingsText = await buildingsRes.text();
        
        // Process Nodes
        const nodesArr = parseCSV(nodesText);
        let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
        
        nodesArr.forEach(n => {
            const x = parseFloat(n.x);
            const y = parseFloat(n.y);
            nodes[n.id] = { id: n.id, x, y };
            minX = Math.min(minX, x); maxX = Math.max(maxX, x);
            minY = Math.min(minY, y); maxY = Math.max(maxY, y);
        });
        
        // Process Edges
        const edgesArr = parseCSV(roadsText);
        edgesArr.forEach(e => {
            edges.push({
                id: e.id,
                source: nodes[e.source],
                target: nodes[e.target],
                lanes: parseInt(e.lanes),
                is_one_way: e.is_one_way === '1'
            });
        });
        
        // Process Buildings
        const bArr = parseCSV(buildingsText);
        bArr.forEach(b => {
            buildings.push({
                name: b.name,
                node_id: b.node_id,
                type: b.type
            });
        });
        
        // Auto-fit camera
        if (minX < Infinity) {
            const margin = 80;
            const zoom = Math.min(
                (canvas.width  - margin * 2) / (maxX - minX),
                (canvas.height - margin * 2) / (maxY - minY),
                1.5
            );
            camera.zoom = zoom;
            camera.x = (canvas.width  - (maxX - minX) * zoom) / 2 - minX * zoom;
            camera.y = (canvas.height - (maxY - minY) * zoom) / 2 - minY * zoom;
        }

        document.getElementById('loading-alert').classList.add('hidden');
        render();

    } catch (err) {
        console.error(err);
        document.getElementById('loading-alert').innerHTML = 
            `Error loading Map Data!<br><small>${err.message}</small>`;
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

// Start
loadMapData();
